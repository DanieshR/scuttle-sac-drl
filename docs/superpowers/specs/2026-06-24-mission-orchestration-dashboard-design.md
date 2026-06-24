# Mission Orchestration + Dashboard Control — Design Spec

**Date:** 2026-06-24
**Branch:** `worktree-physical-deployment`
**Status:** Approved design — pending implementation plan

## Problem

The distributed SCUTTLE stack currently comes up as five hand-run launch files across
two machines (see `deploy/launch/`). The operator wants:

1. The **base stack** to come up automatically per machine (manual/teleop ready).
2. The **frontier autonomy** (Nav2 + explore_lite) to be toggled **from the dashboard**,
   bringing up the extra nodes on *both* machines from one button — and torn down on
   return to manual, leaving the base running.
3. **GDM always running** in the base, with a **dashboard show/hide toggle** for the gas
   heatmap layer (SLAM-only view vs SLAM+heatmap) — display-only, the node keeps running.
4. A way to **save the SLAM map and the GDM map locally**, to an operator-chosen path.

## Existing assets (do not reinvent)

- `deploy/launch/pi_hardware_bringup.launch.py` — Pi sensors/odom/gas/twist_mux (un-gated).
- `deploy/launch/pi_nav2_delayed.launch.py` — Nav2, self-gated on `/map`.
- `deploy/launch/laptop_slam.launch.py` — slam_toolbox, self-gated on `/scan`.
- `deploy/launch/laptop_exploration_gdm.launch.py` — explore_lite + GDM, gated on Nav2.
  **Will be split** (GDM moves to base; this becomes frontier-only).
- `deploy/scripts/wait_for_nav2.py` — Nav2-online gate helper.
- `scuttle_dashboard` (`ros2_ws/src/scuttle_dashboard`) — web SPA + roslib over rosbridge.
  Already has: `setMode('manual'|'auto')` toggle (client-side only today), `/cmd_vel_teleop`,
  `/estop` pub/sub, `/map` + `/odom` + `/gdm/gas_1` subscriptions, and a heatmap subscription
  to `/gdm/gmrf_gas_map` (OccupancyGrid) flagged "confirm topic name".

## Integration gap (must fix)

`gmrf_node` (gmrf_node.cpp:193) publishes `mean_map` / `var_map` as
`sensor_msgs/PointCloud2`, **not** an `OccupancyGrid` named `/gdm/gmrf_gas_map`. The
dashboard heatmap, the show/hide toggle, and the GDM map-save all need a 2D grid.

**Resolution:** add a small **`gmrf_to_grid`** node (Python, rclpy) that subscribes to
`mean_map` (PointCloud2) and republishes `/gdm/gmrf_gas_map` (`nav_msgs/OccupancyGrid`).
Leaves the validated C++ `gmrf_node` untouched. One clean grid topic feeds both the
dashboard heatmap and the map-saver.

## Mission control interface (over rosbridge / Zenoh)

| Name | Type | Dir | Purpose |
|------|------|-----|---------|
| `/mission/mode` | `std_msgs/String` (latched) | dashboard→supervisors | `"manual"` \| `"frontier"` |
| `/mission/state/pi` | `std_msgs/String` (latched) | pi supervisor→dashboard | Pi-side state for chips |
| `/mission/state/laptop` | `std_msgs/String` (latched) | laptop supervisor→dashboard | laptop-side state for chips |
| `/mission/save_path` | `std_msgs/String` (latched) | dashboard→map_saver | operator-chosen base dir |
| `/mission/save_maps` | `std_srvs/Trigger` | dashboard→map_saver | save now; response = written folder |

Latched (`transient_local`) so a late-joining dashboard immediately sees current mode/state.
No custom interface package — keeps rosbridge wiring trivial.

## Architecture

```
Dashboard setMode('auto') ──pub──► /mission/mode ──(Zenoh)──┐
                                                            │
              ┌─────────────────────────────────────────────┴───────────────┐
              ▼                                                               ▼
   pi_mission_supervisor (Pi)                          laptop_mission_supervisor (laptop)
   frontier → spawn pi_nav2_delayed.launch.py          frontier → spawn laptop_frontier.launch.py
   manual   → SIGINT that launch process-group         manual   → SIGINT that launch process-group
              └──────────────► /mission/state ◄────────────────────────────────┘
```

### Always-on base (auto-started)

- **`deploy/launch/pi_base.launch.py`** = `pi_hardware_bringup` + `pi_mission_supervisor`.
- **`deploy/launch/laptop_base.launch.py`** = zenoh router (or run separately) + `laptop_slam`
  + GDM (`map_throttle` + `gmrf_node` + `gmrf_to_grid`) + `scuttle_dashboard` (rosbridge+web)
  + `laptop_mission_supervisor` + `map_saver` node.

### Frontier toggle (spawned by supervisors, never run by hand)

- Pi: `pi_nav2_delayed.launch.py` (unchanged — still self-gates on `/map`).
- **`deploy/launch/laptop_frontier.launch.py`** = explore_lite only (GDM removed; it's in base).

### Supervisor nodes (Python, rclpy) — `deploy/supervisor/`

`mission_supervisor.py`, parameterized by a `launch_file` and a `role` label:
- Subscribe `/mission/mode` (latched). On `"frontier"`: if not already running,
  `subprocess.Popen(['ros2','launch', <file>], start_new_session=True)`. On `"manual"`:
  `os.killpg(SIGINT)` the group, wait, escalate to SIGTERM/SIGKILL on timeout.
- Publish this machine's own latched state topic (`/mission/state/pi` or
  `/mission/state/laptop`) — e.g. `nav2:up`, `frontier:up`, `idle`. Each supervisor owns
  exactly one topic so latched values never clobber each other; the dashboard subscribes to
  both and merges for display.
- Idempotent: re-publish of the same mode is a no-op. Mode change while a spawn/kill is in
  flight is queued, not raced.
- E-STOP interplay: a `frontier→manual` flip kills explore_lite, so no new goals are sent;
  the existing `/estop` path still independently halts motion. (Frontier teardown is the
  graceful stop; `/estop` is the hard stop. Both must work; neither replaces the other.)

### Map saving — `deploy/supervisor/map_saver.py` (laptop)

Backs `/mission/save_maps` (Trigger). On call:
1. Read last `/mission/save_path` (default `~/scuttle_maps`).
2. Make `<base>/<YYYYmmdd-HHMMSS>/`.
3. SLAM: `ros2 run nav2_map_server map_saver_cli -f <dir>/slam_map --ros-args -p map_topic:=/map`.
4. GDM grid: same map_saver against `/gdm/gmrf_gas_map` → `<dir>/gmrf_gas_map.pgm/.yaml`.
5. GDM csv: write the latest grid values to `<dir>/gmrf_gas.csv` (cell x,y,value).
6. Trigger response: `success`, `message=<dir>` so the dashboard can toast the path.

### Dashboard edits (`scuttle_dashboard/web`)

1. **`setMode()` → publish `/mission/mode`** (manual/frontier). The toggle becomes real.
   Keep all existing client-side chip/teleop behavior.
2. **Subscribe `/mission/state/pi` + `/mission/state/laptop`** → merge and drive the autonomy
   chips from real supervisor state instead of the sim's `S.mode` alone.
3. **"Show gas heatmap" toggle** — client-side layer visibility only; `paintGasGrid` is
   gated on a `S.showHeatmap` flag. Does not touch any node.
4. **"Save maps" control** — a path field (prefilled `~/scuttle_maps`) + Save button:
   publish `/mission/save_path`, then call `/mission/save_maps`, toast the returned folder.
5. **Relax boot discovery** (`ros-bindings.js:45`): manual mode has no Nav2/explore yet, so
   require only `slam_toolbox` for "core alive"; show Nav2/explore as "available on frontier".

## Out of scope (YAGNI)

- systemd boot persistence — base is started by one `ros2 launch` per machine for now.
- SAC DRL frontier — explore_lite only; the SAC swap is a later, one-line include change.
- Multi-robot — single robot, unchanged.
- Resumable slam_toolbox pose-graph serialization — only the occupancy grid is saved.

## Testing

- **Supervisor unit:** publish `/mission/mode` manual→frontier→manual; assert the target
  launch PID group appears then is reaped; assert `/mission/state` transitions.
- **gmrf_to_grid:** feed a synthetic `mean_map` PointCloud2; assert a well-formed
  `OccupancyGrid` on `/gdm/gmrf_gas_map` (origin/resolution/dims sane, values 0–100).
- **map_saver:** set `/mission/save_path` to a temp dir, call Trigger; assert
  `slam_map.{pgm,yaml}` + `gmrf_gas_map.{pgm,yaml}` + `gmrf_gas.csv` exist under a timestamp.
- **End-to-end (manual, hotspot):** auto-start both base launches; open dashboard; confirm
  manual teleop + heatmap toggle + save; click AUTONOMOUS, confirm Nav2(Pi)+explore(laptop)
  come up via supervisors and the robot explores; click MANUAL, confirm they tear down and
  the base survives.

## Open risks

- rmw_zenoh + latched (`transient_local`) QoS across the hotspot link — verify the dashboard
  actually receives the latched `/mission/mode`/`/mission/state` on late join.
- Killing a `ros2 launch` group cleanly under zenoh (lifecycle nodes mid-transition) — the
  supervisor must escalate SIGINT→SIGTERM→SIGKILL and confirm child PIDs are gone (prior
  sessions saw zombie nodes inflate the graph; see physical-deployment teardown notes).
