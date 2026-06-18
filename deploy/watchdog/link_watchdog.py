#!/usr/bin/env python3
"""
link_watchdog.py — local link-loss e-stop for the SCUTTLE robot (offload plan, Phase 5).

RUNS ON THE PI, AND ONLY ON THE PI. Its entire job is to notice when the base
station (SLAM / frontier / GDM, reached over the phone hotspot) has gone away and
to stop the robot before Nav2 keeps driving on a stale plan. Every decision and
every actuation is LOCAL — the watchdog never depends on the network it is watching.

DETECTION
  Primary signal: the age of the `map -> odom` transform. SLAM on the base station
  publishes `map -> odom`; it crosses the hotspot. If it stops arriving (link down,
  router dead, base station asleep), the newest `map -> odom` we hold goes stale.
  When its age exceeds `staleness_threshold_s`, OR the transform can't be looked up
  at all, we declare the link DOWN.

  NOTE: comparing a transform stamped on the base station against the Pi's clock is
  only meaningful if the two clocks agree — that is what Phase 1's chrony sync buys
  us. Without chrony, a clock skew would look like permanent staleness. The
  watchdog logs its computed age so a skew is easy to spot.

  Optional secondary signal: a TCP connect to the zenoh router (`router_host` +
  `router_port`). Disabled unless `router_host` is set.

ACTUATION — engage the robot's EXISTING twist_mux e-stop lock
  The robot already runs `twist_mux` (scuttle_base/config/twist_mux.yaml) with a
  LOCK on `/estop` at priority 255 (timeout 0.0 = latched). Engaging that lock
  halts the muxed `cmd_vel` output regardless of what Nav2 (`/cmd_vel_safe`, pri 10)
  or teleop (`/cmd_vel_teleop`, pri 100) is publishing. So the watchdog does NOT
  race /cmd_vel — it just drives the lock:
      link DOWN -> publish std_msgs/Bool(true)  on /estop   (robot held stopped)
      link UP   -> publish std_msgs/Bool(false) on /estop   (control released)
  The lock state is republished at `publish_rate_hz` so twist_mux always holds a
  current value (robust to twist_mux (re)starting after the watchdog). Fail-safe:
  if the watchdog dies while the link is down, the last value on the latched lock
  was `true`, so the robot stays stopped.

RECOVERY
  When the link comes back AND stays healthy for `recovery_debounce_s`, the lock is
  released. The debounce prevents flapping on a marginal link.

RUN
  source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
  python3 deploy/watchdog/link_watchdog.py            # defaults (estop_topic=/estop)
  # or with overrides:
  python3 deploy/watchdog/link_watchdog.py --ros-args \
      -p staleness_threshold_s:=1.5 -p router_host:=<base_ip> -p router_port:=7447
"""

import socket

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException


class LinkWatchdog(Node):
    def __init__(self):
        super().__init__("link_watchdog")

        # --- parameters (all overridable; no hardcoded robot-specific assumptions) ---
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("staleness_threshold_s", 1.5)
        self.declare_parameter("check_rate_hz", 10.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("recovery_debounce_s", 2.0)
        self.declare_parameter("estop_topic", "/estop")     # twist_mux lock topic
        self.declare_parameter("status_topic", "/link_status")
        # Optional secondary probe — leave host empty to disable.
        self.declare_parameter("router_host", "")
        self.declare_parameter("router_port", 7447)
        self.declare_parameter("router_probe_timeout_s", 0.5)

        g = self.get_parameter
        self.map_frame = g("map_frame").value
        self.odom_frame = g("odom_frame").value
        self.staleness_threshold = float(g("staleness_threshold_s").value)
        self.recovery_debounce = float(g("recovery_debounce_s").value)
        self.router_host = g("router_host").value
        self.router_port = int(g("router_port").value)
        self.router_timeout = float(g("router_probe_timeout_s").value)

        check_rate = float(g("check_rate_hz").value)
        publish_rate = float(g("publish_rate_hz").value)

        # --- state ---
        self.link_down = False          # current debounced verdict; also the lock state
        self._healthy_since = None      # time link first looked healthy again

        # --- ROS interfaces ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.estop_pub = self.create_publisher(Bool, g("estop_topic").value, 10)
        self.status_pub = self.create_publisher(Bool, g("status_topic").value, 10)

        self.create_timer(1.0 / check_rate, self._check_link)
        self.create_timer(1.0 / publish_rate, self._publish_lock)

        self.get_logger().info(
            f"link_watchdog up: watching {self.map_frame}->{self.odom_frame}, "
            f"stale>{self.staleness_threshold}s, estop_lock={g('estop_topic').value}, "
            f"router_probe={'on ' + self.router_host if self.router_host else 'off'}. "
            f"ASSUMES chrony-synced clocks (Phase 1)."
        )

    # ---- detection -------------------------------------------------------------

    def _map_odom_age_s(self):
        """Age in seconds of the latest map->odom transform, or None if unavailable."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame, rclpy.time.Time()
            )
        except TransformException:
            return None
        stamp = rclpy.time.Time.from_msg(tf.header.stamp)
        if stamp.nanoseconds == 0:
            return None  # some publishers emit a zero stamp; treat as unusable
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _router_reachable(self):
        """Optional TCP probe of the zenoh router. True if unconfigured (don't fail-closed on it)."""
        if not self.router_host:
            return True
        try:
            with socket.create_connection(
                (self.router_host, self.router_port), timeout=self.router_timeout
            ):
                return True
        except OSError:
            return False

    def _check_link(self):
        age = self._map_odom_age_s()
        tf_ok = age is not None and age <= self.staleness_threshold
        router_ok = self._router_reachable()
        healthy = tf_ok and router_ok

        now = self.get_clock().now()
        self.status_pub.publish(Bool(data=healthy))

        if healthy:
            if self.link_down:
                # candidate recovery — require it to hold for the debounce window
                if self._healthy_since is None:
                    self._healthy_since = now
                elif (now - self._healthy_since) >= Duration(seconds=self.recovery_debounce):
                    self.link_down = False
                    self._healthy_since = None
                    self.get_logger().info("LINK RECOVERED — releasing /estop lock.")
            return

        # unhealthy
        self._healthy_since = None
        if not self.link_down:
            self.link_down = True
            reason = "map->odom unavailable" if age is None else f"map->odom age={age:.2f}s"
            if not router_ok:
                reason += " + zenoh router unreachable"
            self.get_logger().warn(f"LINK DOWN ({reason}) — engaging /estop lock.")

    # ---- actuation -------------------------------------------------------------

    def _publish_lock(self):
        # Republish the lock state every tick so twist_mux always holds a current
        # value (robust to twist_mux restarting). True = locked/stopped.
        self.estop_pub.publish(Bool(data=self.link_down))


def main():
    rclpy.init()
    node = LinkWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("link_watchdog shutting down.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
