#!/usr/bin/env python3
"""
map_saver.py — backs /mission/save_maps (std_srvs/Trigger).

On call: resolve <save_path>/<timestamp>/, run nav2 map_saver_cli for /map (slam_map.*)
and /gdm/gmrf_gas_map (gmrf_gas_map.*), and dump the latest GMRF grid to gmrf_gas.csv.
The destination base dir comes from the latched /mission/save_path the dashboard publishes
(default ~/scuttle_maps). The Trigger response message is the written folder.

  python3 deploy/nodes/map_saver.py
"""

import os
import subprocess
from datetime import datetime


def resolve_save_dir(base_path, now=None):
    """<base or ~/scuttle_maps>/<YYYYmmdd-HHMMSS>, with ~ expanded."""
    now = now or datetime.now()
    base = base_path if base_path else os.path.join(os.path.expanduser('~'), 'scuttle_maps')
    base = os.path.expanduser(base)
    return os.path.join(base, now.strftime('%Y%m%d-%H%M%S'))


def grid_to_csv_rows(width, height, resolution, origin_x, origin_y, data):
    """Per-cell (x_m, y_m, value) at cell centers; row-major matching OccupancyGrid.data."""
    rows = []
    for i, v in enumerate(data):
        gx = i % width
        gy = i // width
        x_m = origin_x + (gx + 0.5) * resolution
        y_m = origin_y + (gy + 0.5) * resolution
        rows.append((x_m, y_m, v))
    return rows


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                           QoSReliabilityPolicy, QoSHistoryPolicy)
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from nav_msgs.msg import OccupancyGrid

    latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    def save_one(topic, stem, out_dir):
        """map_saver_cli for a single OccupancyGrid topic; returns True on success."""
        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
               '-f', os.path.join(out_dir, stem),
               '--ros-args',
               '-r', f'map:={topic}',
               '-p', 'map_subscribe_transient_local:=true',
               '-p', 'save_map_timeout:=10.0']
        return subprocess.run(cmd, timeout=30).returncode == 0

    class MapSaver(Node):
        def __init__(self):
            super().__init__('map_saver')
            self._save_path = None
            self._grid = None
            self.create_subscription(String, '/mission/save_path', self._on_path, latched)
            self.create_subscription(OccupancyGrid, '/gdm/gmrf_gas_map', self._on_grid, latched)
            self.create_service(Trigger, '/mission/save_maps', self._on_save)
            self.get_logger().info('map_saver ready: /mission/save_maps')

        def _on_path(self, msg):
            self._save_path = msg.data

        def _on_grid(self, msg):
            self._grid = msg

        def _on_save(self, request, response):
            out_dir = resolve_save_dir(self._save_path)
            try:
                os.makedirs(out_dir, exist_ok=True)
            except OSError as e:
                response.success = False
                response.message = f'cannot create {out_dir}: {e}'
                return response

            ok_slam = save_one('/map', 'slam_map', out_dir)
            ok_gas = save_one('/gdm/gmrf_gas_map', 'gmrf_gas_map', out_dir)

            if self._grid is not None:
                info = self._grid.info
                rows = grid_to_csv_rows(info.width, info.height, info.resolution,
                                        info.origin.position.x, info.origin.position.y,
                                        list(self._grid.data))
                with open(os.path.join(out_dir, 'gmrf_gas.csv'), 'w') as f:
                    f.write('x_m,y_m,value\n')
                    for x, y, v in rows:
                        f.write(f'{x:.3f},{y:.3f},{v}\n')

            response.success = bool(ok_slam)
            response.message = out_dir if ok_slam else f'{out_dir} (SLAM save failed)'
            self.get_logger().info(f'saved maps -> {out_dir} (slam={ok_slam}, gas={ok_gas})')
            return response

    rclpy.init(args=argv)
    node = MapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
