#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert, Tomas
# Gazebo Harmonic adaptation: SCUTTLE project

import os
import random
import math
import subprocess
import numpy
import time

from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node

from turtlebot3_msgs.srv import RingGoal
import xml.etree.ElementTree as ET
from ..drl_environment.drl_environment import ARENA_LENGTH, ARENA_WIDTH, ENABLE_DYNAMIC_GOALS
from ..common.settings import ENABLE_TRUE_RANDOM_GOALS

# Maps stage number → Gazebo world name (must match <world name="..."> in the SDF)
STAGE_WORLD_NAMES = {
    1: 'drl_stage1', 2: 'drl_stage2', 3: 'drl_stage3',
    4: 'drl_stage4', 5: 'drl_stage5', 6: 'drl_stage6',
    7: 'drl_stage7', 8: 'drl_stage8', 9: 'drl_stage9',
}

# Stages with no inner obstacles — goal placement accepts all arena positions
STAGES_NO_INNER_WALLS = {1, 2, 3, 6}

# Stages that use inner_walls_large instead of inner_walls
STAGES_INNER_WALLS_LARGE = {7, 8, 9}

NO_GOAL_SPAWN_MARGIN = 0.3  # meters away from any wall


class DRLGazebo(Node):
    def __init__(self):
        super().__init__('drl_gazebo')

        with open('/tmp/drlnav_current_stage.txt', 'r') as f:
            self.stage = int(f.read())
        self.world_name = STAGE_WORLD_NAMES.get(self.stage, 'drl_stage4')
        print(f"running on stage: {self.stage}, world: {self.world_name}, dynamic goals enabled: {ENABLE_DYNAMIC_GOALS}")

        self.prev_x, self.prev_y = -1, -1
        self.goal_x, self.goal_y = 0.5, 0.0

        # Initialise publishers
        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pose_pub = self.create_publisher(Pose, 'goal_pose', latch_qos)
        # RViz display: PoseStamped in odom frame so the goal marker is visible.
        # Uses 'odom' frame — matches the corrected TF from odom_to_tf.py.
        self.goal_viz_pub = self.create_publisher(PoseStamped, 'drl_goal_viz', latch_qos)
        self.goal_marker_pub = self.create_publisher(Marker, 'goal_marker', latch_qos)

        # Initialise servers
        self.task_succeed_server = self.create_service(RingGoal, 'task_succeed', self.task_succeed_callback)
        self.task_fail_server    = self.create_service(RingGoal, 'task_fail',    self.task_fail_callback)

        self.obstacle_coordinates = self.get_obstacle_coordinates()
        self.init_callback()

    def init_callback(self):
        self._hide_goal()
        self.reset_simulation()
        self.publish_callback()
        print("Init, goal pose:", self.goal_x, self.goal_y)
        time.sleep(1)

    def publish_callback(self):
        goal_pose = Pose()
        goal_pose.position.x = self.goal_x
        goal_pose.position.y = self.goal_y
        self.goal_pose_pub.publish(goal_pose)

        viz = PoseStamped()
        viz.header.stamp = self.get_clock().now().to_msg()
        viz.header.frame_id = 'odom'
        viz.pose.position.x = self.goal_x
        viz.pose.position.y = self.goal_y
        viz.pose.orientation.w = 1.0
        self.goal_viz_pub.publish(viz)

        self._show_goal()

    def task_succeed_callback(self, request, response):
        # No hide — marker jumps directly to next goal for seamless visual
        if ENABLE_TRUE_RANDOM_GOALS:
            self.generate_random_goal()
            print(f"success: generate (random) a new goal, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        elif ENABLE_DYNAMIC_GOALS:
            self.generate_dynamic_goal_pose(request.robot_pose_x, request.robot_pose_y, request.radius)
            print(f"success: generate a new goal, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}, radius: {request.radius:.2f}")
        else:
            self.generate_goal_pose()
            print(f"success: generate a new goal, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        return response

    def task_fail_callback(self, request, response):
        self._hide_goal()
        self.reset_simulation()
        if ENABLE_TRUE_RANDOM_GOALS:
            self.generate_random_goal()
            print(f"fail: reset the environment, (random) goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        elif ENABLE_DYNAMIC_GOALS:
            self.generate_dynamic_goal_pose(request.robot_pose_x, request.robot_pose_y, request.radius)
            print(f"fail: reset the environment, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}, radius: {request.radius:.2f}")
        else:
            self.generate_goal_pose()
            print(f"fail: reset the environment, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        return response

    def goal_is_valid(self, goal_x, goal_y):
        if goal_x > ARENA_LENGTH/2 or goal_x < -ARENA_LENGTH/2 or goal_y > ARENA_WIDTH/2 or goal_y < -ARENA_WIDTH/2:
            return False
        for obstacle in self.obstacle_coordinates:
            if goal_x < obstacle[0][0] and goal_x > obstacle[2][0]:
                if goal_y < obstacle[0][1] and goal_y > obstacle[2][1]:
                    return False
        return True

    def generate_random_goal(self):
        self.prev_x = self.goal_x
        self.prev_y = self.goal_y
        tries = 0
        while (((abs(self.prev_x - self.goal_x) + abs(self.prev_y - self.goal_y)) < 4) or
               (not self.goal_is_valid(self.goal_x, self.goal_y))):
            self.goal_x = random.randrange(-25, 25) / 10.0
            self.goal_y = random.randrange(-25, 25) / 10.0
            tries += 1
            if tries > 200:
                print("ERROR: cannot find valid new goal, resetting!")
                self._hide_goal()
                self.reset_simulation()
                self.generate_goal_pose()
                break
        self.publish_callback()

    def generate_dynamic_goal_pose(self, robot_pose_x, robot_pose_y, radius):
        tries = 0
        while True:
            ring_position = random.uniform(0, 1)
            origin = radius + numpy.random.normal(0, 0.1)
            goal_offset_x = math.cos(2 * math.pi * ring_position) * origin
            goal_offset_y = math.sin(2 * math.pi * ring_position) * origin
            goal_x = robot_pose_x + goal_offset_x
            goal_y = robot_pose_y + goal_offset_y
            if self.goal_is_valid(goal_x, goal_y):
                self.goal_x = goal_x
                self.goal_y = goal_y
                break
            if tries > 100:
                print("Error! couldn't find valid goal position, resetting..")
                self._hide_goal()
                self.reset_simulation()
                self.generate_goal_pose()
                return
            tries += 1
        self.publish_callback()

    def generate_goal_pose(self):
        self.prev_x = self.goal_x
        self.prev_y = self.goal_y
        tries = 0

        while ((abs(self.prev_x - self.goal_x) + abs(self.prev_y - self.goal_y)) < 2):
            if self.stage == 11:
                goal_pose_list = [[0.0, 0.0], [0.0, 6.5], [5.0, 5.5], [-2.5, -6.0], [3.0, -4.0], [6.0, -1.0]]
                index = random.randrange(0, len(goal_pose_list))
                self.goal_x = float(goal_pose_list[index][0])
                self.goal_y = float(goal_pose_list[index][1])
            elif self.stage in [8, 9, 12]:
                goal_pose_list = [[2.0, 2.0], [2.0, 1.5], [2.0, -0.5], [2.0, -1.0], [2.0, -2.0], [1.3, 1.0],
                                  [1.0, 0.3], [1.0, -2.0], [0.3, -1.0], [0.0, 2.0], [0.0, -1.0], [-1.0, 1.0],
                                  [-1.0, -1.2], [-2.0, 1.0], [-2.2, 0.0], [-2.0, -2.2], [-2.4, 2.4]]
                index = random.randrange(0, len(goal_pose_list))
                self.goal_x = float(goal_pose_list[index][0])
                self.goal_y = float(goal_pose_list[index][1])
            elif self.stage not in [4, 5, 7]:
                self.goal_x = random.randrange(-15, 16) / 10.0
                self.goal_y = random.randrange(-15, 16) / 10.0
            else:
                goal_pose_list = [[1.0, 0.0], [2.0, -1.5], [0.0, -2.0], [2.0, 2.0], [0.8, 2.0],
                                  [-1.9, 1.9], [-1.9, 0.2], [-1.9, -0.5], [-2.0, -2.0], [-0.5, -1.0],
                                  [1.5, -1.0], [-0.5, 1.0], [-1.0, -2.0], [1.8, -0.2], [1.0, -1.9]]
                index = random.randrange(0, len(goal_pose_list))
                self.goal_x = float(goal_pose_list[index][0])
                self.goal_y = float(goal_pose_list[index][1])
            tries += 1
            if tries > 100:
                print("ERROR: distance between goals is small!")
                break
        self.publish_callback()

    # ------------------------------------------------------------------ #
    # Gazebo Harmonic service helpers                                      #
    # ------------------------------------------------------------------ #

    def _gz_set_pose(self, name, x, y, z=0.0):
        """Move a named model via gz service set_pose (Gazebo Harmonic)."""
        req = (
            f'name: "{name}", '
            f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
            f'orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
        )
        subprocess.run(
            ['gz', 'service', '-s', f'/world/{self.world_name}/set_pose',
             '--reqtype', 'gz.msgs.Pose',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000',
             '--req', req],
            capture_output=True,
        )

    def _show_goal(self):
        """Publish a green cylinder marker at the goal position (RViz)."""
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'drl_goal'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        self.goal_marker_pub.publish(marker)

    def _hide_goal(self):
        """Delete the goal marker (RViz)."""
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'drl_goal'
        marker.id = 0
        marker.action = Marker.DELETE
        self.goal_marker_pub.publish(marker)

    def reset_simulation(self):
        """Reset simulation for next episode (Gazebo Harmonic).

        reset:{all:true} reloads the SDF and removes dynamically spawned entities
        (like the robot spawned via ros_gz_sim create) — do NOT use it.
        reset:{model_only:true} resets all model poses + plugin states (including
        diff-drive odometry) without destroying dynamic entities.
        """
        # Teleport robot back to spawn position first so set_pose succeeds
        # even if model_only reset takes a moment to settle.
        self._gz_set_pose('scuttle', 0.0, 0.0, 0.04175)
        # Reset all model poses and diff-drive odom via model_only WorldControl.
        subprocess.run(
            ['gz', 'service', '-s', f'/world/{self.world_name}/control',
             '--reqtype', 'gz.msgs.WorldControl',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000',
             '--req', 'reset: { model_only: true }'],
            capture_output=True,
        )
        time.sleep(0.3)

    # Keep old names as aliases so any internal callers still work
    def delete_entity(self):
        self._hide_goal()

    def spawn_entity(self):
        self._show_goal()

    def get_obstacle_coordinates(self):
        # Stages with no inner walls → no goal exclusion zones needed
        if self.stage in STAGES_NO_INNER_WALLS:
            return []

        if self.stage in STAGES_INNER_WALLS_LARGE:
            model_name = 'inner_walls_large'
        else:
            model_name = 'inner_walls'

        tree = ET.parse(
            os.getenv('DRLNAV_BASE_PATH') +
            f'/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_drl_world/{model_name}/model.sdf'
        )
        root = tree.getroot()
        obstacle_coordinates = []
        for wall in root.find('model').findall('link'):
            pose = wall.find('pose').text.split(" ")
            size = wall.find('collision').find('geometry').find('box').find('size').text.split()
            rotation = float(pose[-1])
            pose_x = float(pose[0])
            pose_y = float(pose[1])
            if rotation == 0:
                size_x = float(size[0]) + NO_GOAL_SPAWN_MARGIN * 2
                size_y = float(size[1]) + NO_GOAL_SPAWN_MARGIN * 2
            else:
                size_x = float(size[1]) + NO_GOAL_SPAWN_MARGIN * 2
                size_y = float(size[0]) + NO_GOAL_SPAWN_MARGIN * 2
            point_1 = [pose_x + size_x / 2, pose_y + size_y / 2]
            point_2 = [point_1[0], point_1[1] - size_y]
            point_3 = [point_1[0] - size_x, point_1[1] - size_y]
            point_4 = [point_1[0] - size_x, point_1[1]]
            obstacle_coordinates.append([point_1, point_2, point_3, point_4])
        return obstacle_coordinates


def main():
    rclpy.init()
    drl_gazebo = DRLGazebo()
    rclpy.spin(drl_gazebo)
    drl_gazebo.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
