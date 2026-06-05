#!/usr/bin/env python3
"""
Standalone DRL inference node for the real SCUTTLE robot.

Subscribes: /scan (LD19, 360 rays), /odom, /goal_pose (geometry_msgs/Pose)
Publishes:  /cmd_vel at 10 Hz
Stops when: goal reached or collision threshold hit

Model: actor_stage9_episode2200 — SAC trained in Gazebo Classic, 60-ray 12m lidar,
4.2x4.2m arena. Best checkpoint by 100-episode window reward across Stage 9.

Usage:
  ros2 run turtlebot3_drl drl_inference
  # or with a custom model:
  DRL_MODEL_PATH=/path/to/actor.pt ros2 run turtlebot3_drl drl_inference
"""
import copy
import math
import os
import sys

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# ── Network: layer names must match training checkpoint keys ──────────────────
_LOG_STD_MIN = -20
_LOG_STD_MAX = 2

class _ActorNet(nn.Module):
    def __init__(self, state_size: int, action_size: int, hidden_size: int):
        super().__init__()
        self.fc1     = nn.Linear(state_size, hidden_size)
        self.fc2     = nn.Linear(hidden_size, hidden_size)
        self.mean    = nn.Linear(hidden_size, action_size)
        self.log_std = nn.Linear(hidden_size, action_size)  # not used at inference

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        return torch.tanh(self.mean(x))  # deterministic greedy action in [-1, 1]


# ── Constants matching training (settings.py in the Docker image) ─────────────
_STATE_SIZE       = 64     # 60 scan + dist_to_goal + goal_angle + prev_lin + prev_ang
_ACTION_SIZE      = 2
_HIDDEN_SIZE      = 512
_LIDAR_IN_RAYS    = 360    # LD19 T-Mini Plus full 360°
_TRAIN_SCAN_BINS  = 60     # training used 60-ray SDF model
_LIDAR_CAP        = 12.0   # metres — training normalization cap
# Real LD19 has a 0.40 m mounting offset; sim needs no correction.
# Override with DRL_LIDAR_CORRECTION=0.0 for sim, 0.40 for real robot.
_LIDAR_CORRECTION = float(os.environ.get('DRL_LIDAR_CORRECTION', '0.0'))
_ARENA_DIAG       = math.sqrt(4.2**2 + 4.2**2)  # ~5.94 m, used to normalize goal dist
_LIN_MAX          = 0.35   # m/s — training SPEED_LINEAR_MAX
_ANG_MAX          = 2.0    # rad/s — training SPEED_ANGULAR_MAX
_GOAL_THRESH      = 0.35   # m — stop when closer than this
_COLL_THRESH      = 0.25   # m — stop when obstacle closer than this
_CONTROL_HZ       = 10.0

_DEFAULT_MODEL = os.path.join(
    os.path.dirname(__file__),
    '..', '..', 'model',
    'daniesh-Legion-Pro-5-16ARX8', 'sac_0_stage_8',
    'actor_stage8_episode1300.pt',
)


class DrlInferenceNode(Node):
    def __init__(self, model_path: str):
        super().__init__('drl_inference')

        device_str = 'cuda' if torch.cuda.is_available() else 'cpu'
        self._device = torch.device(device_str)

        self._actor = _ActorNet(_STATE_SIZE, _ACTION_SIZE, _HIDDEN_SIZE).to(self._device)
        self._actor.load_state_dict(
            torch.load(os.path.realpath(model_path), map_location=self._device)
        )
        self._actor.eval()
        self.get_logger().info(f'Actor loaded from {model_path} ({device_str})')

        # sensor state
        self._scan   = [_LIDAR_CAP] * _TRAIN_SCAN_BINS
        self._obs_dist = _LIDAR_CAP
        self._rx = self._ry = self._heading = 0.0
        self._gx = self._gy = 0.0
        self._goal_dist  = _ARENA_DIAG
        self._goal_angle = 0.0
        self._prev_lin   = 0.0
        self._prev_ang   = 0.0
        self._active     = False

        qos = QoSProfile(depth=10)
        self._cmd_pub  = self.create_publisher(Twist, 'cmd_vel', qos)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self._odom_cb, qos)
        self.create_subscription(Pose, 'goal_pose', self._goal_cb, qos)
        self.create_timer(1.0 / _CONTROL_HZ, self._step)

    # ── callbacks ────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        bucket = n / _TRAIN_SCAN_BINS
        corrected = []
        for i in range(_TRAIN_SCAN_BINS):
            lo, hi = int(i * bucket), int((i + 1) * bucket)
            vals = []
            for j in range(lo, hi):
                r = float(msg.ranges[j])
                if math.isinf(r) or math.isnan(r):
                    r = _LIDAR_CAP
                vals.append(max(0.0, r - _LIDAR_CORRECTION))
            corrected.append(min(vals) if vals else _LIDAR_CAP)

        obs = _LIDAR_CAP
        for i, v in enumerate(corrected):
            self._scan[i] = float(np.clip(v / _LIDAR_CAP, 0.0, 1.0))
            if v < obs:
                obs = v
        self._obs_dist = obs

    def _odom_cb(self, msg: Odometry):
        self._rx = msg.pose.pose.position.x
        self._ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._heading = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._update_goal_relative()

    def _goal_cb(self, msg: Pose):
        self._gx, self._gy = msg.position.x, msg.position.y
        self._active = True
        self._prev_lin = self._prev_ang = 0.0
        self._update_goal_relative()
        self.get_logger().info(f'New goal → ({self._gx:.2f}, {self._gy:.2f})')

    # ── helpers ───────────────────────────────────────────────────────────────

    def _update_goal_relative(self):
        dx, dy = self._gx - self._rx, self._gy - self._ry
        self._goal_dist = math.sqrt(dx * dx + dy * dy)
        angle = math.atan2(dy, dx) - self._heading
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        self._goal_angle = angle

    def _stop(self):
        self._active = False
        self._cmd_pub.publish(Twist())

    # ── control loop ─────────────────────────────────────────────────────────

    def _step(self):
        if not self._active:
            return

        if self._goal_dist < _GOAL_THRESH:
            self.get_logger().info('Goal reached.')
            self._stop()
            return

        if self._obs_dist < _COLL_THRESH:
            self.get_logger().warn(
                f'Collision threshold ({self._obs_dist:.2f} m < {_COLL_THRESH} m), stopping.'
            )
            self._stop()
            return

        state = copy.copy(self._scan)
        state.append(float(np.clip(self._goal_dist / _ARENA_DIAG, 0.0, 1.0)))
        state.append(float(self._goal_angle) / math.pi)
        state.append(self._prev_lin)
        state.append(self._prev_ang)

        with torch.no_grad():
            a = self._actor(
                torch.FloatTensor(state).unsqueeze(0).to(self._device)
            ).squeeze(0).cpu().numpy()

        # ENABLE_BACKWARD=False: map [-1,1] → [0, LIN_MAX] for forward-only
        linear  = float((a[0] + 1.0) / 2.0 * _LIN_MAX)
        angular = float(a[1] * _ANG_MAX)

        self._prev_lin = float(a[0])
        self._prev_ang = float(a[1])

        twist = Twist()
        twist.linear.x  = linear
        twist.angular.z = angular
        self._cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    model_path = os.environ.get('DRL_MODEL_PATH', _DEFAULT_MODEL)
    node = DrlInferenceNode(model_path)
    try:
        rclpy.spin(node)
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
