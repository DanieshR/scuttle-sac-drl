#!/usr/bin/env python3
"""
gmrf_to_grid.py — non-invasive bridge: GMRF_mean.csv -> /gdm/gmrf_gas_map OccupancyGrid.

gmrf_node publishes nothing on a topic (its PointCloud2 path is commented out); it only
writes GMRF_mean.csv when output_csv_folder is set. This watches that CSV, takes the grid
origin from the latest /map, the resolution from the cell_size param, and republishes an
OccupancyGrid the dashboard heatmap + map_saver can consume.

  python3 deploy/nodes/gmrf_to_grid.py --ros-args \
      -p csv_path:=/run/scuttle/gmrf/GMRF_mean.csv -p cell_size:=0.5
"""

import math

# ----------------------- pure, import-only logic -----------------------

def parse_mean_csv(text):
    """GMRF_mean.csv -> list of rows (list[float|None]); blank/NaN -> None."""
    rows = []
    for line in text.strip().splitlines():
        line = line.strip()
        if not line:
            continue
        cells = []
        for tok in line.split(','):
            tok = tok.strip()
            if tok == '' or tok.lower() == 'nan':
                cells.append(None)
                continue
            try:
                v = float(tok)
            except ValueError:
                cells.append(None)
                continue
            cells.append(None if math.isnan(v) else v)
        rows.append(cells)
    return rows


def mean_to_int8(mean):
    """Normalized mean [0,1] -> OccupancyGrid int8 [0,100]; None -> -1 (unknown)."""
    if mean is None:
        return -1
    return max(0, min(100, int(round(mean * 100.0))))


def flatten_occupancy(rows, flip_vertical=True):
    """Row-major OccupancyGrid.data + (width, height).

    GMRF writes y increasing downward (row 0 on top); OccupancyGrid.data starts at the
    origin (bottom-left) with y increasing up, so flip vertically by default. Ragged rows
    are padded to max width with unknown (-1)."""
    if not rows:
        return [], 0, 0
    height = len(rows)
    width = max(len(r) for r in rows)
    ordered = list(reversed(rows)) if flip_vertical else rows
    data = []
    for r in ordered:
        for x in range(width):
            cell = r[x] if x < len(r) else None
            data.append(mean_to_int8(cell))
    return data, width, height


# ----------------------------- rclpy wrapper -----------------------------

def main(argv=None):
    import os
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                           QoSReliabilityPolicy, QoSHistoryPolicy)
    from nav_msgs.msg import OccupancyGrid

    latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    class GmrfToGrid(Node):
        def __init__(self):
            super().__init__('gmrf_to_grid')
            self.declare_parameter('csv_path', os.path.expanduser('~/.scuttle/gmrf/GMRF_mean.csv'))
            self.declare_parameter('cell_size', 0.5)
            self.declare_parameter('output_topic', '/gdm/gmrf_gas_map')
            self.declare_parameter('map_topic', '/map')
            self.declare_parameter('frame_id', 'map')
            self.declare_parameter('period_sec', 1.0)
            self._csv = self.get_parameter('csv_path').value
            self._cell = float(self.get_parameter('cell_size').value)
            self._frame = self.get_parameter('frame_id').value
            self._origin = None
            self.pub = self.create_publisher(
                OccupancyGrid, self.get_parameter('output_topic').value, latched)
            self.create_subscription(
                OccupancyGrid, self.get_parameter('map_topic').value, self._on_map, latched)
            self.create_timer(float(self.get_parameter('period_sec').value), self._tick)
            self.get_logger().info(f'gmrf_to_grid watching {self._csv}')

        def _on_map(self, msg):
            self._origin = (msg.info.origin.position.x, msg.info.origin.position.y)

        def _tick(self):
            if self._origin is None or not os.path.exists(self._csv):
                return
            try:
                with open(self._csv, 'r') as f:
                    rows = parse_mean_csv(f.read())
            except OSError:
                return
            data, w, h = flatten_occupancy(rows, flip_vertical=True)
            if w == 0 or h == 0:
                return
            g = OccupancyGrid()
            g.header.frame_id = self._frame
            g.header.stamp = self.get_clock().now().to_msg()
            g.info.resolution = self._cell
            g.info.width = w
            g.info.height = h
            g.info.origin.position.x = self._origin[0]
            g.info.origin.position.y = self._origin[1]
            g.info.origin.orientation.w = 1.0
            g.data = data
            self.pub.publish(g)

    rclpy.init(args=argv)
    node = GmrfToGrid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
