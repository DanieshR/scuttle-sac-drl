# Mission Orchestration + Dashboard Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the distributed SCUTTLE stack start hands-off per machine and be driven entirely from the web dashboard — manual/frontier mode toggle, always-on GDM with a heatmap show/hide, and operator-path map saving.

**Architecture:** Three small standalone `rclpy` scripts (`gmrf_to_grid`, `mission_supervisor`, `map_saver`) live in `deploy/nodes/` and are launched via `ExecuteProcess` (same pattern as the existing `deploy/scripts/wait_for_nav2.py`). Each isolates its logic in pure, import-only functions that are unit-tested without a ROS graph; the thin rclpy wrapper lives in `main()`. The dashboard's existing `setMode()` toggle becomes the orchestrator trigger over `/mission/mode`; supervisors spawn/kill the frontier launch trees. systemd brings the base up on boot.

**Tech Stack:** ROS 2 Jazzy, rclpy, `rmw_zenoh`, `nav2_map_server` (map_saver_cli), slam_toolbox, explore_lite, rosbridge_suite, roslib.js, systemd, pytest.

## Global Constraints

- ROS 2 distro: **Jazzy**. Real hardware → `use_sim_time:=false` everywhere; never set true.
- Every shell/process that touches the ROS graph must inherit a `zenoh-env.sh`-sourced env (handled by the launch parent / systemd unit; tests of pure functions do NOT need it).
- Do **not** edit the C++ `gmrf_gas_mapping` package. `gmrf_to_grid` is non-invasive.
- GMRF gas scale is **0–110** sensor units, but `GMRF_mean.csv` values are normalized **[0,1]**; OccupancyGrid carries **0–100** (int8), unknown = **-1**.
- Mission topics, exact names: `/mission/mode` (`std_msgs/String`), `/mission/state/pi` + `/mission/state/laptop` (`std_msgs/String`), `/mission/save_path` (`std_msgs/String`), `/mission/save_maps` (`std_srvs/Trigger`). Latched topics use `TRANSIENT_LOCAL` durability, depth 1.
- GMRF grid topic: `/gdm/gmrf_gas_map` (`nav_msgs/OccupancyGrid`), published `TRANSIENT_LOCAL`.
- Pi systemd = **system** service; laptop systemd = **user** services (`loginctl enable-linger`). No kiosk browser.
- Python tests run from repo root: `cd ~/ros2_ws && python3 -m pytest deploy/tests/<file> -v`.

---

## File Structure

**Create:**
- `deploy/nodes/gmrf_to_grid.py` — CSV→OccupancyGrid bridge (pure builders + rclpy wrapper).
- `deploy/nodes/mission_supervisor.py` — mode→launch process supervisor (pure decision + ProcessManager + wrapper).
- `deploy/nodes/map_saver.py` — `/mission/save_maps` service (pure path/csv helpers + wrapper).
- `deploy/tests/conftest.py` — puts `deploy/nodes` on `sys.path`.
- `deploy/tests/test_gmrf_to_grid.py`, `test_mission_supervisor.py`, `test_map_saver.py`, `test_launch_files.py`.
- `deploy/launch/pi_base.launch.py`, `laptop_base.launch.py`, `laptop_frontier.launch.py`.
- `deploy/systemd/scuttle-base.service.in`, `zenoh-router.service.in`, `scuttle-laptop-base.service.in`, `install.sh`, `uninstall.sh`.

**Modify:**
- `src/scuttle_dashboard/web/ros-bindings.js` — publish `/mission/mode`, subscribe state topics, save flow, heatmap gate, relax boot discovery.
- `src/scuttle_dashboard/web/index.html` — heatmap toggle + save control + `setMode` wiring.

**Delete:**
- `deploy/launch/laptop_exploration_gdm.launch.py` — superseded (GDM → base, frontier → `laptop_frontier.launch.py`).

---

### Task 1: `gmrf_to_grid` — CSV→OccupancyGrid bridge

**Files:**
- Create: `deploy/nodes/gmrf_to_grid.py`
- Create: `deploy/tests/conftest.py`
- Test: `deploy/tests/test_gmrf_to_grid.py`

**Interfaces:**
- Produces (pure, importable): `parse_mean_csv(text: str) -> list[list[float|None]]`; `mean_to_int8(mean: float|None) -> int`; `flatten_occupancy(rows: list[list[float|None]], flip_vertical: bool=True) -> tuple[list[int], int, int]` returning `(data, width, height)`.
- Runtime: subscribes `/map` (origin), reads `GMRF_mean.csv`, publishes `/gdm/gmrf_gas_map` (`nav_msgs/OccupancyGrid`, TRANSIENT_LOCAL).

- [ ] **Step 1: Write the conftest so tests can import the nodes**

Create `deploy/tests/conftest.py`:

```python
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'nodes'))
```

- [ ] **Step 2: Write the failing test**

Create `deploy/tests/test_gmrf_to_grid.py`:

```python
from gmrf_to_grid import parse_mean_csv, mean_to_int8, flatten_occupancy


def test_parse_mean_csv_basic():
    rows = parse_mean_csv("0.0,0.5\n1.0,\n")
    assert rows == [[0.0, 0.5], [1.0, None]]


def test_parse_mean_csv_handles_nan_and_blanks():
    rows = parse_mean_csv("nan, ,0.25")
    assert rows == [[None, None, 0.25]]


def test_mean_to_int8_scales_and_clamps():
    assert mean_to_int8(None) == -1
    assert mean_to_int8(0.0) == 0
    assert mean_to_int8(0.5) == 50
    assert mean_to_int8(1.0) == 100
    assert mean_to_int8(1.7) == 100      # clamp high
    assert mean_to_int8(-0.3) == 0       # clamp low


def test_flatten_occupancy_dims_and_flip():
    rows = [[0.0, 0.0], [1.0, 1.0]]      # row 0 = top in CSV
    data, w, h = flatten_occupancy(rows, flip_vertical=True)
    assert (w, h) == (2, 2)
    # flipped: bottom row (originally row 1) comes first in OccupancyGrid.data
    assert data == [100, 100, 0, 0]


def test_flatten_occupancy_ragged_rows_padded_unknown():
    rows = [[0.5], [0.5, 0.5]]
    data, w, h = flatten_occupancy(rows, flip_vertical=False)
    assert (w, h) == (2, 2)
    assert data == [50, -1, 50, 50]      # first row padded with unknown
```

- [ ] **Step 3: Run test to verify it fails**

Run: `cd ~/ros2_ws && python3 -m pytest deploy/tests/test_gmrf_to_grid.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'gmrf_to_grid'`.

- [ ] **Step 4: Write the node (pure functions + rclpy wrapper)**

Create `deploy/nodes/gmrf_to_grid.py`:

```python
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
```

- [ ] **Step 5: Run test to verify it passes**

Run: `cd ~/ros2_ws && python3 -m pytest deploy/tests/test_gmrf_to_grid.py -v`
Expected: PASS (5 passed).

- [ ] **Step 6: Commit**

```bash
cd ~/ros2_ws
git add deploy/nodes/gmrf_to_grid.py deploy/tests/conftest.py deploy/tests/test_gmrf_to_grid.py
git commit -m "feat(deploy): gmrf_to_grid CSV->OccupancyGrid bridge"
```

---

### Task 2: `mission_supervisor` — mode→launch process supervisor

**Files:**
- Create: `deploy/nodes/mission_supervisor.py`
- Test: `deploy/tests/test_mission_supervisor.py`

**Interfaces:**
- Produces (pure): `decide_action(running: bool, requested: str) -> str` returning `'start' | 'stop' | 'noop'`.
- Produces (class): `ProcessManager(cmd: list[str])` with `.running` (bool property), `.start()`, `.stop(sigint_timeout=10.0, sigterm_timeout=5.0)`.
- Runtime: subscribes `/mission/mode` (latched), publishes `--state-topic` (latched). CLI args: `--launch-file <path>`, `--role <pi|laptop>`, `--state-topic <topic>`, `--mode-topic /mission/mode`.

- [ ] **Step 1: Write the failing test**

Create `deploy/tests/test_mission_supervisor.py`:

```python
import time

from mission_supervisor import decide_action, ProcessManager


def test_decide_action_matrix():
    assert decide_action(running=False, requested='frontier') == 'start'
    assert decide_action(running=True,  requested='frontier') == 'noop'
    assert decide_action(running=True,  requested='manual')   == 'stop'
    assert decide_action(running=False, requested='manual')   == 'noop'
    assert decide_action(running=False, requested='idle')     == 'noop'  # any non-frontier


def test_process_manager_start_and_stop():
    pm = ProcessManager(['sleep', '1000'])
    assert pm.running is False
    pm.start()
    assert pm.running is True
    pm.start()                       # idempotent — no second process
    assert pm.running is True
    pm.stop(sigint_timeout=5.0, sigterm_timeout=2.0)
    assert pm.running is False


def test_process_manager_stop_when_not_running_is_safe():
    pm = ProcessManager(['sleep', '1000'])
    pm.stop()                        # should not raise
    assert pm.running is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd ~/ros2_ws && python3 -m pytest deploy/tests/test_mission_supervisor.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'mission_supervisor'`.

- [ ] **Step 3: Write the node**

Create `deploy/nodes/mission_supervisor.py`:

```python
#!/usr/bin/env python3
"""
mission_supervisor.py — turns /mission/mode into a launched (or killed) frontier stack.

One instance per machine. On '/mission/mode' == 'frontier' it spawns its --launch-file as
a new process group; on anything else it SIGINT->SIGTERM->SIGKILL escalates that group.
Publishes a per-machine latched state string so the dashboard can show real status.

  python3 deploy/nodes/mission_supervisor.py \
      --role pi --launch-file /home/amr/ros2_ws/deploy/launch/pi_nav2_delayed.launch.py \
      --state-topic /mission/state/pi
"""

import argparse
import os
import signal
import subprocess
import time


def decide_action(running, requested):
    """requested in {'manual','frontier','idle',...}; returns 'start'|'stop'|'noop'."""
    if requested == 'frontier':
        return 'noop' if running else 'start'
    return 'stop' if running else 'noop'


class ProcessManager:
    """Owns a single child process group; start is idempotent, stop escalates signals."""

    def __init__(self, cmd):
        self.cmd = cmd
        self.proc = None

    @property
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self):
        if self.running:
            return
        self.proc = subprocess.Popen(self.cmd, start_new_session=True)

    def stop(self, sigint_timeout=10.0, sigterm_timeout=5.0):
        if not self.running:
            self.proc = None
            return
        pgid = os.getpgid(self.proc.pid)
        for sig, timeout in ((signal.SIGINT, sigint_timeout),
                             (signal.SIGTERM, sigterm_timeout),
                             (signal.SIGKILL, 2.0)):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                break
            if self._wait(timeout):
                break
        self.proc = None

    def _wait(self, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if not self.running:
                return True
            time.sleep(0.1)
        return not self.running


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--role', required=True)
    parser.add_argument('--launch-file', required=True)
    parser.add_argument('--state-topic', required=True)
    parser.add_argument('--mode-topic', default='/mission/mode')
    args, _ = parser.parse_known_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                           QoSReliabilityPolicy, QoSHistoryPolicy)
    from std_msgs.msg import String

    latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    class MissionSupervisor(Node):
        def __init__(self):
            super().__init__(f'mission_supervisor_{args.role}')
            self.pm = ProcessManager(['ros2', 'launch', args.launch_file])
            self.state_pub = self.create_publisher(String, args.state_topic, latched)
            self.create_subscription(String, args.mode_topic, self._on_mode, latched)
            self._publish_state('idle')

        def _publish_state(self, s):
            self.state_pub.publish(String(data=s))

        def _on_mode(self, msg):
            action = decide_action(self.pm.running, msg.data.strip())
            if action == 'start':
                self.get_logger().info(f'[{args.role}] frontier requested -> launching')
                self._publish_state('frontier:starting')
                self.pm.start()
                self._publish_state('frontier:up')
            elif action == 'stop':
                self.get_logger().info(f'[{args.role}] manual requested -> tearing down')
                self._publish_state('frontier:stopping')
                self.pm.stop()
                self._publish_state('idle')

    rclpy.init(args=argv)
    node = MissionSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pm.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd ~/ros2_ws && python3 -m pytest deploy/tests/test_mission_supervisor.py -v`
Expected: PASS (3 passed). The start/stop test takes ~1s.

- [ ] **Step 5: Commit**

```bash
cd ~/ros2_ws
git add deploy/nodes/mission_supervisor.py deploy/tests/test_mission_supervisor.py
git commit -m "feat(deploy): mission_supervisor mode->launch process supervisor"
```

---

### Task 3: `map_saver` — `/mission/save_maps` service

**Files:**
- Create: `deploy/nodes/map_saver.py`
- Test: `deploy/tests/test_map_saver.py`

**Interfaces:**
- Produces (pure): `resolve_save_dir(base_path: str|None, now: datetime) -> str` → `<base or ~/scuttle_maps>/<YYYYmmdd-HHMMSS>`; `grid_to_csv_rows(width, height, resolution, origin_x, origin_y, data: list[int]) -> list[tuple[float,float,int]]`.
- Runtime: subscribes `/mission/save_path` (latched, String) and `/gdm/gmrf_gas_map` (latched, OccupancyGrid); serves `/mission/save_maps` (`std_srvs/Trigger`); shells `ros2 run nav2_map_server map_saver_cli`.

- [ ] **Step 1: Write the failing test**

Create `deploy/tests/test_map_saver.py`:

```python
from datetime import datetime

from map_saver import resolve_save_dir, grid_to_csv_rows


def test_resolve_save_dir_with_base():
    d = resolve_save_dir('/tmp/maps', datetime(2026, 6, 24, 14, 5, 9))
    assert d == '/tmp/maps/20260624-140509'


def test_resolve_save_dir_default(monkeypatch):
    monkeypatch.setenv('HOME', '/home/tester')
    d = resolve_save_dir(None, datetime(2026, 6, 24, 14, 5, 9))
    assert d == '/home/tester/scuttle_maps/20260624-140509'


def test_resolve_save_dir_empty_string_is_default(monkeypatch):
    monkeypatch.setenv('HOME', '/home/tester')
    d = resolve_save_dir('', datetime(2026, 1, 2, 3, 4, 5))
    assert d == '/home/tester/scuttle_maps/20260102-030405'


def test_grid_to_csv_rows_centers_cells():
    # 2x1 grid, 0.5 m cells, origin (1.0, 2.0), values [40, 60]
    rows = grid_to_csv_rows(2, 1, 0.5, 1.0, 2.0, [40, 60])
    assert rows == [
        (1.25, 2.25, 40),
        (1.75, 2.25, 60),
    ]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd ~/ros2_ws && python3 -m pytest deploy/tests/test_map_saver.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'map_saver'`.

- [ ] **Step 3: Write the node**

Create `deploy/nodes/map_saver.py`:

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd ~/ros2_ws && python3 -m pytest deploy/tests/test_map_saver.py -v`
Expected: PASS (4 passed).

- [ ] **Step 5: Commit**

```bash
cd ~/ros2_ws
git add deploy/nodes/map_saver.py deploy/tests/test_map_saver.py
git commit -m "feat(deploy): map_saver service for /mission/save_maps"
```

---

### Task 4: Launch files — `laptop_frontier`, `pi_base`, `laptop_base`

**Files:**
- Create: `deploy/launch/laptop_frontier.launch.py`
- Create: `deploy/launch/pi_base.launch.py`
- Create: `deploy/launch/laptop_base.launch.py`
- Delete: `deploy/launch/laptop_exploration_gdm.launch.py`
- Test: `deploy/tests/test_launch_files.py`

**Interfaces:**
- Consumes: `deploy/launch/pi_hardware_bringup.launch.py`, `pi_nav2_delayed.launch.py`, `laptop_slam.launch.py` (existing); `scuttle_nav/launch/frontier.launch.py`; `scuttle_dashboard/launch/dashboard.launch.py`; nodes from Tasks 1–3.
- Produces: each module exposes `generate_launch_description() -> LaunchDescription`.

- [ ] **Step 1: Write `laptop_frontier.launch.py` (explore_lite only)**

Create `deploy/launch/laptop_frontier.launch.py`:

```python
#!/usr/bin/env python3
"""
laptop_frontier.launch.py — spawned by laptop_mission_supervisor on frontier mode.

Just explore_lite (real clock). GDM lives in laptop_base now; Nav2 is on the Pi. The
supervisor gates this on /mission/mode, and explore_lite itself only acts once Nav2's
action server is up, so no extra gate is needed here.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_nav = get_package_share_directory('scuttle_nav')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'frontier.launch.py')),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),
    ])
```

- [ ] **Step 2: Write `pi_base.launch.py`**

Create `deploy/launch/pi_base.launch.py`:

```python
#!/usr/bin/env python3
"""
pi_base.launch.py — the Pi's always-on base. Auto-started by scuttle-base.service.

Hardware bringup (sensors/odom/gas/twist_mux) + the mission supervisor that brings up
pi_nav2_delayed.launch.py when the dashboard requests frontier mode.
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEPLOY = os.path.dirname(_HERE)


def generate_launch_description():
    supervisor = os.path.join(_DEPLOY, 'nodes', 'mission_supervisor.py')
    nav2_launch = os.path.join(_HERE, 'pi_nav2_delayed.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(_HERE, 'pi_hardware_bringup.launch.py'))),
        ExecuteProcess(
            cmd=['python3', supervisor,
                 '--role', 'pi',
                 '--launch-file', nav2_launch,
                 '--state-topic', '/mission/state/pi'],
            name='mission_supervisor_pi', output='screen'),
    ])
```

- [ ] **Step 3: Write `laptop_base.launch.py`**

Create `deploy/launch/laptop_base.launch.py`:

```python
#!/usr/bin/env python3
"""
laptop_base.launch.py — the laptop's always-on base. Auto-started by scuttle-laptop-base.service.

SLAM + GDM (map_throttle + gmrf_node writing CSV + gmrf_to_grid bridge) + the dashboard
(rosbridge+web) + mission supervisor (brings up laptop_frontier on frontier mode) + map_saver.
The zenoh router runs as its own systemd service, started before this launch.
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEPLOY = os.path.dirname(_HERE)
_GMRF_CSV_DIR = os.path.expanduser('~/.scuttle/gmrf')


def generate_launch_description():
    pkg_dashboard = get_package_share_directory('scuttle_dashboard')
    supervisor = os.path.join(_DEPLOY, 'nodes', 'mission_supervisor.py')
    gmrf_to_grid = os.path.join(_DEPLOY, 'nodes', 'gmrf_to_grid.py')
    map_saver = os.path.join(_DEPLOY, 'nodes', 'map_saver.py')
    frontier_launch = os.path.join(_HERE, 'laptop_frontier.launch.py')

    os.makedirs(_GMRF_CSV_DIR, exist_ok=True)

    return LaunchDescription([
        # SLAM (gated on /scan)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(_HERE, 'laptop_slam.launch.py'))),

        # GDM — always on. map_throttle feeds gmrf_node; gmrf_node writes CSV; bridge republishes.
        Node(package='scuttle_base', executable='map_throttle', name='map_throttle',
             output='screen',
             parameters=[{'input_topic': '/map', 'output_topic': '/map_throttled',
                          'period_sec': 30.0, 'use_sim_time': False}]),
        Node(package='gmrf_gas_mapping', executable='gmrf_node', name='gmrf_node',
             output='screen',
             parameters=[{'use_sim_time': False, 'frame_id': 'map',
                          'occupancy_map_topic': '/map_throttled', 'sensor_topic': '/gdm/gas_1',
                          'observation_topic': '/gdm/obs_unused',
                          'output_csv_folder': _GMRF_CSV_DIR,
                          'exec_freq': 2.0, 'cell_size': 0.5,
                          'min_sensor_val': 0.0, 'max_sensor_val': 110.0,
                          'GMRF_lambdaPrior': 5.0, 'GMRF_lambdaObs': 0.5,
                          'GMRF_lambdaObsLoss': 0.01}]),
        ExecuteProcess(
            cmd=['python3', gmrf_to_grid, '--ros-args',
                 '-p', f'csv_path:={os.path.join(_GMRF_CSV_DIR, "GMRF_mean.csv")}',
                 '-p', 'cell_size:=0.5'],
            name='gmrf_to_grid', output='screen'),

        # Dashboard (rosbridge + static web)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dashboard, 'launch', 'dashboard.launch.py'))),

        # Mission supervisor + map saver
        ExecuteProcess(
            cmd=['python3', supervisor,
                 '--role', 'laptop',
                 '--launch-file', frontier_launch,
                 '--state-topic', '/mission/state/laptop'],
            name='mission_supervisor_laptop', output='screen'),
        ExecuteProcess(cmd=['python3', map_saver], name='map_saver', output='screen'),
    ])
```

- [ ] **Step 4: Delete the superseded launch file**

```bash
cd ~/ros2_ws
git rm deploy/launch/laptop_exploration_gdm.launch.py
```

- [ ] **Step 5: Write the launch-validity test**

Create `deploy/tests/test_launch_files.py`:

```python
"""Launch files must import and produce a LaunchDescription.

Run in a ROS-sourced shell (needs launch/launch_ros/ament_index):
    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
    python3 -m pytest deploy/tests/test_launch_files.py -v
"""
import importlib.util
import os

from launch import LaunchDescription

LAUNCH_DIR = os.path.join(os.path.dirname(__file__), '..', 'launch')


def _load(name):
    path = os.path.join(LAUNCH_DIR, name)
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_laptop_frontier_builds():
    ld = _load('laptop_frontier.launch.py').generate_launch_description()
    assert isinstance(ld, LaunchDescription)


def test_pi_base_builds():
    ld = _load('pi_base.launch.py').generate_launch_description()
    assert isinstance(ld, LaunchDescription)
    assert len(ld.entities) >= 2          # hardware include + supervisor


def test_laptop_base_builds():
    ld = _load('laptop_base.launch.py').generate_launch_description()
    assert isinstance(ld, LaunchDescription)
    assert len(ld.entities) >= 6          # slam + 3 gdm + dashboard + supervisor + map_saver


def test_exploration_gdm_removed():
    assert not os.path.exists(os.path.join(LAUNCH_DIR, 'laptop_exploration_gdm.launch.py'))
```

- [ ] **Step 6: Run the test (ROS-sourced)**

Run: `cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 -m pytest deploy/tests/test_launch_files.py -v`
Expected: PASS (4 passed). If `scuttle_dashboard` isn't built yet, run `colcon build --packages-select scuttle_dashboard` first.

- [ ] **Step 7: Commit**

```bash
cd ~/ros2_ws
git add deploy/launch/laptop_frontier.launch.py deploy/launch/pi_base.launch.py \
        deploy/launch/laptop_base.launch.py deploy/tests/test_launch_files.py
git commit -m "feat(deploy): pi_base/laptop_base/laptop_frontier launch + remove exploration_gdm"
```

---

### Task 5: Dashboard wiring — `setMode`, state, heatmap toggle, save, boot

**Files:**
- Modify: `src/scuttle_dashboard/web/ros-bindings.js`
- Modify: `src/scuttle_dashboard/web/index.html`

**Interfaces:**
- Consumes: mission topics/service from Global Constraints; `/gdm/gmrf_gas_map` from Task 1.
- Produces: real `/mission/mode` publishes from the existing `setMode()`; `window.saveMaps(path)`; `window.toggleHeatmap(on)`.

- [ ] **Step 1: Syntax-check the JS as-is (baseline)**

Run: `cd ~/ros2_ws/src/scuttle_dashboard && node --check web/ros-bindings.js && echo OK`
Expected: `OK` (establishes the file parses before edits).

- [ ] **Step 2: Add mission wiring to `ros-bindings.js`**

In `src/scuttle_dashboard/web/ros-bindings.js`, inside the IIFE after the existing
`estopPub` block (search for `window.fireEstop = function`), add:

```javascript
  // ---- Mission control: mode toggle drives the supervisors on both machines. ----
  const latched = { latch: true };
  const modePub = topic('/mission/mode', 'std_msgs/msg/String', latched);
  const savePathPub = topic('/mission/save_path', 'std_msgs/msg/String', latched);

  // Wrap the existing client-side setMode so the toggle also publishes the real mode.
  const _setMode = window.setMode;
  window.setMode = function (m) {
    _setMode(m);
    if (S.live) modePub.publish(new ROSLIB.Message({ data: m === 'manual' ? 'manual' : 'frontier' }));
  };

  // Per-machine state strings -> merge for the autonomy chip.
  const missionState = { pi: '', laptop: '' };
  function showMissionState() {
    const txt = `pi:${missionState.pi || '?'} · laptop:${missionState.laptop || '?'}`;
    const el = document.getElementById('sacState');
    if (el) el.title = txt;
  }
  topic('/mission/state/pi', 'std_msgs/msg/String', { throttle_rate: 0 })
    .subscribe(m => { missionState.pi = m.data; showMissionState(); });
  topic('/mission/state/laptop', 'std_msgs/msg/String', { throttle_rate: 0 })
    .subscribe(m => { missionState.laptop = m.data; showMissionState(); });

  // Save maps: publish the chosen path, then call the Trigger; toast the returned folder.
  const saveSrv = new ROSLIB.Service({
    ros, name: '/mission/save_maps', serviceType: 'std_srvs/srv/Trigger'
  });
  window.saveMaps = function (path) {
    if (!S.live) { toast('Save needs a live connection'); return; }
    if (path) savePathPub.publish(new ROSLIB.Message({ data: path }));
    setTimeout(() => saveSrv.callService(new ROSLIB.ServiceRequest({}), res => {
      toast(res.success ? ('Maps saved → ' + res.message) : ('Save failed: ' + res.message));
    }, err => toast('Save error: ' + err)), 200);
  };

  // Heatmap layer is client-side only; gmrf_node keeps running regardless.
  window.toggleHeatmap = function (on) { S.showHeatmap = !!on; };
```

- [ ] **Step 3: Gate the heatmap render on `S.showHeatmap`**

In `ros-bindings.js`, find `window.paintGasGrid = function (ctx, grid) {` and make it the
first line of the function body:

```javascript
  window.paintGasGrid = function (ctx, grid) {
    if (S.showHeatmap === false) return;          // operator hid the layer
```

- [ ] **Step 4: Relax boot discovery (manual mode has no Nav2/explore yet)**

In `ros-bindings.js`, change the core-stack check (around line 45) from:

```javascript
    const core = ['slam_toolbox', 'controller_server' /* nav2 */, 'explore'];
    if (nodes.length && !core.every(have)) {
      // graph reachable but stack incomplete → 003·A checklist fallback (don't hard-fail)
      setBoot('45%', '⚠ core stack incomplete · waiting on SLAM / Nav2 / frontier…');
    } else {
```

to:

```javascript
    const core = ['slam_toolbox'];                 // base mode: SLAM only; Nav2/explore arrive on frontier
    if (nodes.length && !core.every(have)) {
      setBoot('45%', '⚠ base stack incomplete · waiting on SLAM…');
    } else {
```

- [ ] **Step 5: Add the heatmap toggle + save control + default `S.showHeatmap` to `index.html`**

In `src/scuttle_dashboard/web/index.html`, add `showHeatmap:true,` to the `S = {...}` state
object (next to `mode:'auto',` near line 429):

```javascript
  mode:'auto', showHeatmap:true, paused:false, connected:true, estop:false, live:false,
```

Then, inside the map panel header (search for `id="mapSac"`), add the heatmap toggle button
right after that chip's closing `</span>`:

```html
      <button class="btn" id="btnHeat" onclick="S.showHeatmap=!S.showHeatmap;this.classList.toggle('on',S.showHeatmap);if(window.toggleHeatmap)window.toggleHeatmap(S.showHeatmap);">Gas heatmap</button>
```

And add a save control near the autonomy panel (search for `id="sacStep"`, add after its
enclosing `.stat` div):

```html
      <div class="stat"><span>Save maps</span>
        <span class="v">
          <input id="savePath" type="text" placeholder="~/scuttle_maps" style="width:120px">
          <button class="btn" onclick="(window.saveMaps||function(){})(document.getElementById('savePath').value)">Save</button>
        </span>
      </div>
```

- [ ] **Step 6: Syntax-check the edited JS and confirm wiring exists**

Run:
```bash
cd ~/ros2_ws/src/scuttle_dashboard
node --check web/ros-bindings.js && \
grep -q "/mission/mode" web/ros-bindings.js && \
grep -q "/mission/save_maps" web/ros-bindings.js && \
grep -q "showHeatmap" web/index.html && echo "WIRING OK"
```
Expected: `WIRING OK`.

- [ ] **Step 7: Commit**

```bash
cd ~/ros2_ws
git add src/scuttle_dashboard/web/ros-bindings.js src/scuttle_dashboard/web/index.html
git commit -m "feat(dashboard): wire mission mode, state, heatmap toggle, save maps"
```

---

### Task 6: systemd autostart units + installer

**Files:**
- Create: `deploy/systemd/scuttle-base.service.in`
- Create: `deploy/systemd/zenoh-router.service.in`
- Create: `deploy/systemd/scuttle-laptop-base.service.in`
- Create: `deploy/systemd/install.sh`
- Create: `deploy/systemd/uninstall.sh`

**Interfaces:**
- Consumes: `deploy/launch/pi_base.launch.py`, `laptop_base.launch.py` (Task 4); `deploy/scripts/zenoh-env.sh`; `rmw_zenohd` router executable.
- Produces: installable units; `install.sh <pi|laptop> <laptop_ip> <ws_path>` templates and enables.

- [ ] **Step 1: Write the Pi system unit template**

Create `deploy/systemd/scuttle-base.service.in`:

```ini
[Unit]
Description=SCUTTLE base (hardware + mission supervisor)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=__USER__
ExecStart=/bin/bash -lc 'set +u; source /opt/ros/jazzy/setup.bash; source __WS__/install/setup.bash; source __WS__/deploy/scripts/zenoh-env.sh pi __LAPTOP_IP__; exec ros2 launch __WS__/deploy/launch/pi_base.launch.py'
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

- [ ] **Step 2: Write the laptop user unit templates**

Create `deploy/systemd/zenoh-router.service.in`:

```ini
[Unit]
Description=rmw_zenoh router (base station)
After=network-online.target

[Service]
Type=simple
ExecStart=/bin/bash -lc 'set +u; source /opt/ros/jazzy/setup.bash; source __WS__/install/setup.bash; exec ros2 run rmw_zenoh_cpp rmw_zenohd'
Restart=always
RestartSec=5

[Install]
WantedBy=default.target
```

Create `deploy/systemd/scuttle-laptop-base.service.in`:

```ini
[Unit]
Description=SCUTTLE laptop base (SLAM + GDM + dashboard + mission)
After=zenoh-router.service network-online.target
Wants=zenoh-router.service

[Service]
Type=simple
ExecStart=/bin/bash -lc 'set +u; source /opt/ros/jazzy/setup.bash; source __WS__/install/setup.bash; source __WS__/deploy/scripts/zenoh-env.sh base; exec ros2 launch __WS__/deploy/launch/laptop_base.launch.py'
Restart=always
RestartSec=5

[Install]
WantedBy=default.target
```

- [ ] **Step 3: Write the installer**

Create `deploy/systemd/install.sh`:

```bash
#!/usr/bin/env bash
# install.sh <pi|laptop> <laptop_ip> [ws_path]
#   pi     -> system service scuttle-base.service (needs sudo)
#   laptop -> user services zenoh-router + scuttle-laptop-base (+ linger)
set -euo pipefail

ROLE="${1:?usage: install.sh <pi|laptop> <laptop_ip> [ws_path]}"
LAPTOP_IP="${2:?need laptop_ip}"
WS="${3:-$HOME/ros2_ws}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

render() { sed -e "s|__USER__|$USER|g" -e "s|__WS__|$WS|g" -e "s|__LAPTOP_IP__|$LAPTOP_IP|g" "$1"; }

if [[ "$ROLE" == "pi" ]]; then
  render "$HERE/scuttle-base.service.in" | sudo tee /etc/systemd/system/scuttle-base.service >/dev/null
  sudo systemctl daemon-reload
  sudo systemctl enable --now scuttle-base.service
  echo "installed system service: scuttle-base.service"
elif [[ "$ROLE" == "laptop" ]]; then
  mkdir -p "$HOME/.config/systemd/user"
  render "$HERE/zenoh-router.service.in"        > "$HOME/.config/systemd/user/zenoh-router.service"
  render "$HERE/scuttle-laptop-base.service.in" > "$HOME/.config/systemd/user/scuttle-laptop-base.service"
  loginctl enable-linger "$USER"
  systemctl --user daemon-reload
  systemctl --user enable --now zenoh-router.service scuttle-laptop-base.service
  echo "installed user services: zenoh-router, scuttle-laptop-base"
else
  echo "unknown role: $ROLE" >&2; exit 1
fi
```

Create `deploy/systemd/uninstall.sh`:

```bash
#!/usr/bin/env bash
# uninstall.sh <pi|laptop>
set -euo pipefail
ROLE="${1:?usage: uninstall.sh <pi|laptop>}"
if [[ "$ROLE" == "pi" ]]; then
  sudo systemctl disable --now scuttle-base.service || true
  sudo rm -f /etc/systemd/system/scuttle-base.service
  sudo systemctl daemon-reload
elif [[ "$ROLE" == "laptop" ]]; then
  systemctl --user disable --now scuttle-laptop-base.service zenoh-router.service || true
  rm -f "$HOME/.config/systemd/user/zenoh-router.service" \
        "$HOME/.config/systemd/user/scuttle-laptop-base.service"
  systemctl --user daemon-reload
else
  echo "unknown role: $ROLE" >&2; exit 1
fi
```

- [ ] **Step 4: Make scripts executable and verify rendered units pass systemd-analyze**

Run:
```bash
cd ~/ros2_ws/deploy/systemd
chmod +x install.sh uninstall.sh
# Render with sample values and verify each unit is well-formed (no install needed):
for u in scuttle-base zenoh-router scuttle-laptop-base; do
  sed -e "s|__USER__|$USER|g" -e "s|__WS__|$HOME/ros2_ws|g" -e "s|__LAPTOP_IP__|10.118.109.47|g" \
      "$u.service.in" > "/tmp/$u.service"
  systemd-analyze verify "/tmp/$u.service" && echo "OK $u"
done
bash -n install.sh && bash -n uninstall.sh && echo "scripts OK"
```
Expected: `OK scuttle-base`, `OK zenoh-router`, `OK scuttle-laptop-base`, `scripts OK`. (`systemd-analyze verify` may warn about unknown users on a different machine — a clean exit with no `Failed` lines is success.)

- [ ] **Step 5: Commit**

```bash
cd ~/ros2_ws
git add deploy/systemd/
git commit -m "feat(deploy): systemd autostart units + installer (pi system / laptop user)"
```

---

### Task 7: End-to-end integration verification (manual, on hardware)

**Files:** none (verification only — record results in the commit message of any fixups).

This task has no automated test; it is the live bring-up gate. Run it on the real Pi+laptop
over the hotspot. Each check is pass/fail.

- [ ] **Step 1: Build the workspace on the laptop**

Run: `cd ~/ros2_ws && colcon build --packages-select scuttle_dashboard scuttle_base gmrf_gas_mapping scuttle_nav && source install/setup.bash`
Expected: build succeeds. Confirm `sudo apt list --installed 2>/dev/null | grep -E 'rosbridge-suite|nav2-map-server'` shows both; if missing, `sudo apt install ros-jazzy-rosbridge-suite ros-jazzy-nav2-map-server`.

- [ ] **Step 2: Install autostart on both machines**

On the Pi: `~/ros2_ws/deploy/systemd/install.sh pi <laptop_ip> ~/ros2_ws`
On the laptop: `~/ros2_ws/deploy/systemd/install.sh laptop <laptop_ip> ~/ros2_ws`
Reboot both. Expected after boot, no terminal:
- Pi: `systemctl is-active scuttle-base` → `active`.
- Laptop: `systemctl --user is-active zenoh-router scuttle-laptop-base` → `active active`.

- [ ] **Step 3: Confirm the cross-machine graph + dashboard**

On the laptop (zenoh-sourced shell): `ros2 topic hz /scan` and `ros2 topic hz /odom` show data from the Pi. Open `http://localhost:8080` → dashboard reaches LIVE (not SIM), map renders, gas heatmap visible.
Expected: `/scan` ~10 Hz, `/odom` ~19 Hz; dashboard LIVE badge.

- [ ] **Step 4: Heatmap toggle + save (manual mode)**

In the dashboard: click "Gas heatmap" → heatmap hides/shows (no node restart in `ros2 node list`). Type a path in the save field, click Save → toast shows `Maps saved → <path>/<timestamp>`. Verify on disk: `ls <path>/<timestamp>/` contains `slam_map.pgm`, `slam_map.yaml`, `gmrf_gas_map.pgm`, `gmrf_gas_map.yaml`, `gmrf_gas.csv`.

- [ ] **Step 5: Frontier toggle up**

Click AUTONOMOUS (FRONTIER). Expected: `ros2 node list` now shows Nav2 lifecycle nodes (Pi) and `explore_node` (laptop); `/mission/state/pi` and `/mission/state/laptop` publish `frontier:up`; the robot begins exploring (wheels clear, e-stop reachable).
Run: `ros2 topic echo /mission/state/pi --once` → `data: frontier:up`.

- [ ] **Step 6: Frontier toggle down → base survives**

Click MANUAL TELEOP. Expected: Nav2 + `explore_node` disappear from `ros2 node list`; `slam_toolbox`, `gmrf_node`, `gmrf_to_grid`, rosbridge remain; teleop drives again; `/mission/state/*` → `idle`. The robot stops taking new autonomous goals.

- [ ] **Step 7: Commit any fixups found during E2E**

```bash
cd ~/ros2_ws
git add -A
git commit -m "fix(deploy): E2E bring-up corrections"   # only if changes were needed
```

---

## Self-Review

**Spec coverage:**
- Mission interface (`/mission/mode`, `/mission/state/{pi,laptop}`, `/mission/save_path`, `/mission/save_maps`) → Tasks 2, 3, 5 (+ Global Constraints).
- Always-on base per machine → Task 4 (`pi_base`, `laptop_base`).
- Frontier toggle spawns/kills both machines → Tasks 2 + 4.
- GDM always-on + `gmrf_to_grid` CSV bridge → Tasks 1 + 4.
- Map saving to operator path → Task 3 + dashboard save control (Task 5).
- Dashboard: setMode publish, state subscribe, heatmap show/hide, save, relaxed boot → Task 5.
- systemd autostart (Pi system / laptop user, no kiosk) → Task 6.
- E2E reliability/teardown checks → Task 7.

**Placeholder scan:** none — every code/config step contains full content; verification steps give exact commands + expected output.

**Type consistency:** `parse_mean_csv`/`mean_to_int8`/`flatten_occupancy` (Task 1) used identically by the Task 1 wrapper. `decide_action`/`ProcessManager.running/start/stop` (Task 2) match the wrapper. `resolve_save_dir`/`grid_to_csv_rows` (Task 3) match. Launch files reference node filenames created in Tasks 1–3 and the existing `pi_hardware_bringup`/`pi_nav2_delayed`/`laptop_slam`. Dashboard `window.setMode`/`saveMaps`/`toggleHeatmap`/`S.showHeatmap` consistent across Task 5 steps.

**Open risk carried from spec:** latched (`TRANSIENT_LOCAL`) QoS for `/mission/*` over the hotspot, and clean launch-group teardown under zenoh — both exercised by Task 7 steps 5–6; if late-join latching misbehaves, fall back to a 1 Hz re-publish of `/mission/mode` from the dashboard.
