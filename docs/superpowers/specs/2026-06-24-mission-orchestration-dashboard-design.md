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

`gmrf_node` publishes **nothing on any topic**. The PointCloud2 publish path in
`publishMaps()` (gmrf_node.cpp:184-195) is entirely commented out (ROS1-era API, never
ported). Its only live output is `CGMRF_map::save_as_CSV()` (gmrf_map.cpp:788), which —
when the `output_csv_folder` param is set — writes `GMRF_mean.csv` and `GMRF_std.csv`
each exec cycle (~2 Hz): a row-major matrix of `m_size_y` rows × `m_size_x` cols, values
normalized to [0,1], **no geometry** (origin/resolution not in the file). The dashboard
heatmap (`ros-bindings.js:119`, subscribing `/gdm/gmrf_gas_map`) has therefore never had
live data.

**Resolution (non-invasive — no C++ change):** add a **`gmrf_to_grid`** node (Python,
rclpy) that:
- watches `GMRF_mean.csv` in the configured `output_csv_folder` (gmrf_node writes it),
- reads the matrix (cols = width, rows = height), scales mean [0,1] → [0,100],
- takes resolution from gmrf's `cell_size` param (0.5) and **origin from the latest `/map`**
  (the GMRF grid is initialized against the occupancy map, so it shares its origin),
- publishes `/gdm/gmrf_gas_map` (`nav_msgs/OccupancyGrid`).

This one clean grid topic feeds both the dashboard heatmap and the map-saver, and leaves
the validated C++ `gmrf_node` untouched. (Alternative considered: port the commented-out
C++ PointCloud2 publisher — rejected as it edits a working package for no gain here.)

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
- **`deploy/launch/laptop_base.launch.py`** = `laptop_slam` + GDM (`map_throttle` +
  `gmrf_node` with `output_csv_folder` set to a runtime dir + `gmrf_to_grid` watching that
  CSV) + `scuttle_dashboard` (rosbridge+web) + `laptop_mission_supervisor` + `map_saver`
  node. (The zenoh router runs as its own systemd service, started before this launch —
  see Autostart.)

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

## Autostart — fully hands-off, no terminal (`deploy/systemd/`)

Goal: power on both machines → entire base comes up → operate solely from the dashboard.

**Pi — system-level services** (headless robot, boot before login):
- `scuttle-base.service`: `After=network-online.target`; `ExecStart` sources
  `/opt/ros/jazzy/setup.bash` + `~/ros2_ws/install/setup.bash` + `zenoh-env.sh pi <laptop_ip>`,
  then `ros2 launch deploy/launch/pi_base.launch.py`. `Restart=always`, `RestartSec=5`.
  Depends on stable udev names already in place (`/dev/ttyESP32`, `/dev/ttyAIoT`, lidar).

**Laptop — user-level services** (live in the graphical session; `systemctl --user`,
`loginctl enable-linger` so they start at boot even pre-login):
- `zenoh-router.service`: `ExecStart` runs the rmw_zenoh router (`rmw_zenohd`).
- `scuttle-laptop-base.service`: `After=zenoh-router.service`; sources ROS +
  `zenoh-env.sh base`, runs `ros2 launch deploy/launch/laptop_base.launch.py`
  (SLAM + GDM + gmrf_to_grid + scuttle_dashboard + supervisor + map_saver). `Restart=always`.

The dashboard is served at `http://localhost:8080`; the operator opens the URL in a browser
(no kiosk autostart — explicit decision). Frontier/heatmap/save are all driven from the page.

Install via a `deploy/systemd/install.sh` that templates `<laptop_ip>`/paths, copies units,
`daemon-reload`, and enables. Uninstall script symmetric. Units are templates, not committed
with machine-specific IPs hardcoded.

**Reliability posture (honest):** unattended in the normal case via `Restart=always`. The
fragile boot-time dependencies over the phone hotspot are (1) Pi WiFi auto-association
(NetworkManager autoconnect + `network-online.target`; zenoh peers retry regardless),
(2) serial enumeration (ride over with `Restart`), (3) mid-mission link loss (watchdog
asserts `/estop`; services auto-restart on return). systemd cannot power the machines on —
that single physical action remains the operator's.

## Out of scope (YAGNI)

- Kiosk browser autostart — operator opens the served URL manually (explicit decision).
- SAC DRL frontier — explore_lite only; the SAC swap is a later, one-line include change.
- Multi-robot — single robot, unchanged.
- Resumable slam_toolbox pose-graph serialization — only the occupancy grid is saved.

## Testing

- **Supervisor unit:** publish `/mission/mode` manual→frontier→manual; assert the target
  launch PID group appears then is reaped; assert `/mission/state` transitions.
- **gmrf_to_grid:** given a synthetic `GMRF_mean.csv` matrix + a reference `/map` origin,
  assert the pure builder returns a well-formed `OccupancyGrid` (width=cols, height=rows,
  resolution=cell_size, origin=map origin, values 0–100, NaN/blank→-1).
- **map_saver:** set `/mission/save_path` to a temp dir, call Trigger; assert
  `slam_map.{pgm,yaml}` + `gmrf_gas_map.{pgm,yaml}` + `gmrf_gas.csv` exist under a timestamp.
- **Autostart (reboot):** reboot both machines (no terminal); confirm `scuttle-base` (Pi),
  `zenoh-router` + `scuttle-laptop-base` (laptop) reach `active`, the dashboard answers at
  `localhost:8080`, and the Pi's `/scan`/`/odom` are visible to the laptop over Zenoh — all
  without a manual `ros2 launch`.
- **End-to-end (manual, hotspot):** with both base stacks up, open dashboard; confirm
  manual teleop + heatmap toggle + save; click AUTONOMOUS, confirm Nav2(Pi)+explore(laptop)
  come up via supervisors and the robot explores; click MANUAL, confirm they tear down and
  the base survives.

## Open risks

- rmw_zenoh + latched (`transient_local`) QoS across the hotspot link — verify the dashboard
  actually receives the latched `/mission/mode`/`/mission/state` on late join.
- Killing a `ros2 launch` group cleanly under zenoh (lifecycle nodes mid-transition) — the
  supervisor must escalate SIGINT→SIGTERM→SIGKILL and confirm child PIDs are gone (prior
  sessions saw zombie nodes inflate the graph; see physical-deployment teardown notes).
