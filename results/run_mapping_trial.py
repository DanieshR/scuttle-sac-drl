"""
Mapping trial runner — Section 3.2/3.3 data collection.
Subscribes to /map and /odom, records coverage and trajectory for one trial.

Usage (run AFTER launching sim + agent):
  python3 run_mapping_trial.py --method sac_frontier --trial 1 --duration 180
  python3 run_mapping_trial.py --method dwa_frontier --trial 1 --duration 180
  python3 run_mapping_trial.py --method random_walk  --trial 1 --duration 180

Run 5 trials per method, then call:
  python3 analyze_mapping_results.py
"""

import argparse
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist

OUT = os.path.expanduser("~/Desktop/results/mapping_trials")
os.makedirs(OUT, exist_ok=True)

COLLISION_RANGE_M = 0.25  # distance threshold for "near-miss" (matches training)


class TrialRecorder(Node):
    def __init__(self, method: str, trial: int, duration: float):
        super().__init__("trial_recorder")
        self.method   = method
        self.trial    = trial
        self.duration = duration
        self.start_t  = None

        self.odom_log      = []   # [(t, x, y)]
        self.coverage_log  = []   # [(t, coverage_pct)]
        self.collision_cnt = 0
        self.prev_pos      = None

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(OccupancyGrid, "/map",     self._map_cb,  map_qos)
        self.create_subscription(Odometry,      "/odom",    self._odom_cb, 10)

        # for random walk: publish cmd_vel
        if method == "random_walk":
            self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.create_timer(0.2, self._random_walk_cb)
            self._import_random()

        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f"Trial recorder started: method={method} trial={trial} duration={duration}s"
        )

    def _import_random(self):
        import random as _r
        self._rand = _r

    def _random_walk_cb(self):
        msg = Twist()
        msg.linear.x  = self._rand.uniform(0.0, 0.20)
        msg.angular.z = self._rand.uniform(-1.5, 1.5)
        self.cmd_pub.publish(msg)

    def _odom_cb(self, msg):
        if self.start_t is None:
            return
        t = time.time() - self.start_t
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.odom_log.append((t, x, y))

    def _map_cb(self, msg):
        if self.start_t is None:
            return
        t = time.time() - self.start_t
        data = msg.data
        total_free = sum(1 for c in data if c == 0)
        observed   = sum(1 for c in data if c >= 0)   # not unknown (-1)
        pct = 100.0 * total_free / max(observed, 1)
        self.coverage_log.append((t, pct))

    def _tick(self):
        if self.start_t is None:
            self.start_t = time.time()
            self.get_logger().info("Trial started.")
            return
        elapsed = time.time() - self.start_t
        if elapsed >= self.duration:
            self._save()
            rclpy.shutdown()

    def _save(self):
        stem = f"{OUT}/{self.method}_trial{self.trial:02d}"

        # odom → trajectory CSV
        with open(f"{stem}_trajectory.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "x_m", "y_m"])
            w.writerows(self.odom_log)

        # coverage CSV
        with open(f"{stem}_coverage.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "coverage_pct"])
            w.writerows(self.coverage_log)

        # path length
        path_length = 0.0
        for i in range(1, len(self.odom_log)):
            dx = self.odom_log[i][1] - self.odom_log[i-1][1]
            dy = self.odom_log[i][2] - self.odom_log[i-1][2]
            path_length += math.sqrt(dx*dx + dy*dy)

        # coverage at checkpoints
        def cov_at(t_target):
            entries = [(t, c) for t, c in self.coverage_log if t <= t_target]
            return entries[-1][1] if entries else 0.0

        # time to X% coverage
        def time_to(pct_target):
            for t, c in self.coverage_log:
                if c >= pct_target:
                    return t
            return None

        meta = {
            "method":          self.method,
            "trial":           self.trial,
            "duration_s":      self.duration,
            "coverage_60s":    cov_at(60),
            "coverage_120s":   cov_at(120),
            "coverage_180s":   cov_at(180),
            "time_to_50pct":   time_to(50),
            "time_to_75pct":   time_to(75),
            "time_to_90pct":   time_to(90),
            "path_length_m":   round(path_length, 3),
            "n_odom_samples":  len(self.odom_log),
        }

        with open(f"{stem}_meta.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=meta.keys())
            w.writeheader()
            w.writerow(meta)

        self.get_logger().info(f"Saved trial data to {stem}_*.csv")
        self.get_logger().info(
            f"  Coverage@60s={meta['coverage_60s']:.1f}%  "
            f"  Coverage@120s={meta['coverage_120s']:.1f}%  "
            f"  Path={path_length:.2f}m  "
            f"  Time@90%={meta['time_to_90pct']}s"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--method",   required=True,
                        choices=["sac_frontier", "dwa_frontier", "random_walk"])
    parser.add_argument("--trial",    type=int, default=1)
    parser.add_argument("--duration", type=float, default=180.0,
                        help="Trial duration in seconds")
    args = parser.parse_args()

    rclpy.init()
    node = TrialRecorder(args.method, args.trial, args.duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._save()
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
