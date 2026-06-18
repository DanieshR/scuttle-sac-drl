# Phase 0 — Baseline snapshot (current single-machine setup)

Captured read-only from the robot **`amr@10.118.109.73`** (`amr-pi`, ROS 2 Jazzy)
via `tar`-over-`ssh`. This is the source material for the Phase 3 node split — the
robot-side launch/param files below do **not** exist on the base-station laptop
(its `~/ros2_ws` is a diverged clone), so they were pulled from the Pi.

> The robot's ROS stack was **not running** at capture time, so the live
> `ros2 node list` graph and the `view_frames` TF tree are not included. Capture
> those during the first live bring-up (Phase 4) with `ros2 node list`,
> `ros2 topic list`, and `ros2 run tf2_tools view_frames`. The static launch +
> param capture here is sufficient to drive the Phase 3 split.

## Today: everything runs on the Pi

| Workload | Launch (on Pi) | Key topics / TF |
|---|---|---|
| **Base / motor / odom** | `scuttle_base/launch/base.launch.py` → `scuttle_base_node` | serial `/dev/ttyESP32` @115200; `publish_tf:True` ⇒ **`odom`→`base_link` published locally**; consumes muxed `cmd_vel` |
| **LiDAR** | `ydlidar_ws/.../launch/ydlidar.py` → `ydlidar_ros2_driver_node` | `/dev/ttyLIDAR` @230400 (TminiPlus.yaml); `/scan`; static TF `base_link`→`laser_frame` |
| **cmd_vel arbitration** | `scuttle_base/config/twist_mux.yaml` (`twist_mux`) | inputs: `/cmd_vel_teleop` (pri 100), `/cmd_vel_safe` (pri 10); **lock `/estop` (pri 255)** |
| **SLAM** | `scuttle/.../scuttle_nav/launch/slam.launch.py` (`mapper_params_online_async.yaml`) | slam_toolbox async ⇒ **`map`→`odom`**, `/map` |
| **Nav2** | `scuttle/.../scuttle_nav/launch/nav2.launch.py` (`nav2_params.yaml`) | controller = **RegulatedPurePursuitController** (FollowPath); `navigate_to_pose` |
| **Frontier** | `scuttle/.../m-explore-ros2/explore/launch/explore.launch.py` | explore_lite; consumes `/map`, sends `navigate_to_pose` goals |
| **GDM** | `scuttle_base/launch/gdm.launch.py` | `fake_gas_sensor`→`/gdm/gas_1`; `map_throttle` `/map`→`/map_throttled` (30 s); `gmrf_node` consumes both |

## Target split (Phase 3) — where each piece goes

- **Stays on Pi** (must survive a flaky link): base/motor/odom (`odom`→`base_link`
  local), LiDAR driver, twist_mux + `/estop`, **full Nav2**.
- **Moves to base station** (consumes `/scan`+`/tf`, produces `map`→`odom`/`/map`):
  SLAM, frontier, GDM.

## Real-hardware facts that correct the laptop's stale `CLAUDE.md`

- **Nav2 controller is RegulatedPurePursuitController**, not MPPI (verified in the
  Pi's `nav2_params.yaml` → `FollowPath` plugin). RPP is the right choice for the Pi.
- **Devices are udev symlinks**, not raw `ttyUSBn`: motor `/dev/ttyESP32` (ESP32
  serial, 115200), LiDAR `/dev/ttyLIDAR` (230400). Stable across reboots.
- **Kinematics:** `wheel_separation=0.4247 m`, `wheel_radius=0.04175 m`,
  `counts_per_rev=16384` (from `base.launch.py` + `robot_params.yaml`).
- **LiDAR is the YDLidar T-mini Plus** via `ydlidar_ros2_driver` (TminiPlus.yaml:
  sample_rate 4, intensity on, range_max 12.0) — not `rplidar_ros`. The `rplidar`
  block in `robot_params.yaml` is stale/sim-oriented and unused on real hardware.
- **GDM is already wired** (resolves the open question in the offload notes): the
  MAPIRlab `gmrf_node` runs with real params injected by `gdm.launch.py`
  (`sensor_topic:=/gdm/gas_1`, `occupancy_map_topic:=/map_throttled`, `frame_id:=map`,
  `use_sim_time:=False`) — the placeholder values in `gmrf_params.yaml` are overridden.

## TF contract for the split (must hold)

```
map ──(SLAM, base station, crosses hotspot)──► odom ──(scuttle_base, Pi, local)──► base_link
                                                          └─ static ─► laser_frame
```

`odom`→`base_link` never crosses the network (motor loop stays real-time on the Pi);
only `map`→`odom` does. This is the property that makes the offload safe.

## Inventory captured under `deploy/baseline/pi/`

- `scuttle_base/` — `base.launch.py`, `gdm.launch.py`, `config/twist_mux.yaml`
- `scuttle/src/scuttle_bringup/` — `launch/` (real, robot, exploration, standby, slam, sim, view) + `config/` (robot_params, slam_params)
- `scuttle/src/scuttle_nav/` — `launch/` (slam, nav2, frontier) + `config/` (nav2_params, mapper_params_online_async, explore_params)
- `ydlidar_ws/.../ydlidar_ros2_driver/` — `launch/` + `params/TminiPlus.yaml`
