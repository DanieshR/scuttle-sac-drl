# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace overview

ROS2 Jazzy workspace for a **SCUTTLE differential-drive robot** doing autonomous goal-navigation using SAC deep reinforcement learning, with a GDM gas-sensor pipeline planned for Phase 2.

**Stack:**
- Simulation: Gazebo Harmonic (gz-sim 8.x) + ros-gz bridge
- Mapping: SLAM Toolbox (online async)
- Navigation safety layer: Nav2 (MPPI controller, collision-avoidance only — not used as planner during RL training)
- DRL: `turtlebot3_drlnav` framework (tomasvr/turtlebot3_drlnav) — DDPG/TD3/DQN, 360° LiDAR, multi-node architecture
- Frontier exploration baseline: `explore_lite` (m-explore-ros2)
- Gas simulation: GADEN2 + olfaction_msgs
- Gas distribution mapping: GDM scripts in `~/ros2_ws/gdm/`

## Environment setup

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
# Both are already in ~/.bashrc — new terminals get them automatically.
```

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Incompatible packages are permanently excluded via `COLCON_IGNORE` files in their directories — no `--packages-skip` needed:

- `turtlebot3_gazebo` — classic Gazebo package (inside turtlebot3_drlnav), not for Harmonic
- `turtlebot3_fake_node` — uses `declare_parameter(name)` (no default), removed in Jazzy rclcpp
- `pygaden2` / `pygaden2_demo` — pybind11 CMake incompatibility on Ubuntu 24.04
- `turtlebot3_simulations` — classic Gazebo, models/SDF files are used as data only (not built)
- `pygaden2` / `pygaden2_demo` — pybind11 CMake incompatibility on Ubuntu 24.04; the core `gaden2` and `gaden2_rviz` C++ packages build fine without them

Build a single package:
```bash
colcon build --symlink-install --packages-select <pkg_name>
```

## Package layout

```
src/
  scuttle_description/          # URDF/xacro, meshes, bridge_config.yaml (symlink → ~/Desktop)
  scuttle_gazebo/               # Worlds (.sdf), gz-sim launch, odom_to_tf.py, GasSourcePlugin.cpp
  scuttle_bringup/              # Top-level launch files (sim.launch.py, drl_tomasvr.launch.py, real.launch.py)
  scuttle_nav/                  # SLAM Toolbox + Nav2 launch, maps/, config/
  m-explore-ros2/               # explore_lite frontier exploration
  turtlebot3_drlnav/            # DRL framework (tomasvr) — turtlebot3_drl + turtlebot3_msgs packages
  gaden2/                       # Gas dispersion simulation (BAMresearch fork)
  olfaction_msgs/               # ROS2 msg types for gas sensors
gdm/                            # Gas Distribution Mapping Python scripts (not a ROS pkg)
```

**Desktop packages** (`~/Desktop/scuttle_*`) are the canonical source — the `src/` entries are symlinks.

## Common launch commands

### Simulation
```bash
# Full simulation (Gazebo + robot + bridge + odom→TF)
ros2 launch scuttle_bringup sim.launch.py
ros2 launch scuttle_bringup sim.launch.py world:=gas_maze   # gas world

# SLAM mapping
ros2 launch scuttle_nav slam.launch.py

# Nav2 (safety layer only)
ros2 launch scuttle_nav nav2.launch.py

# Frontier exploration
ros2 launch scuttle_nav frontier.launch.py

# Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Monitoring
```bash
ros2 topic hz /scan          # should be ~10 Hz (LD19)
ros2 topic hz /map           # ~1 Hz from SLAM Toolbox
ros2 topic echo /cmd_vel     # confirm SAC is publishing
```

### Save map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/scuttle_nav/maps/my_map
```

### GADEN2 gas simulation
```bash
ros2 launch gaden2 gaden_player.launch.py scenario:=office use_sim_time:=true
```

## DRL training (`src/turtlebot3_drlnav/` — tomasvr framework)

Four-terminal training stack. **Set env var first** (if not in ~/.bashrc):
```bash
export DRLNAV_BASE_PATH=/home/daniesh/ros2_ws/src/turtlebot3_drlnav
echo "4" > /tmp/drlnav_current_stage.txt
```

```bash
# Terminal 1 — Gazebo simulation (headless)
ros2 launch scuttle_bringup drl_tomasvr.launch.py

# Terminal 2 — goal manager
ros2 run turtlebot3_drl gazebo_goals

# Terminal 3 — environment node (state/reward/done)
ros2 run turtlebot3_drl environment

# Terminal 4 — training agent (td3 | ddpg | dqn)
ros2 run turtlebot3_drl train_agent td3

# To test a saved model:
ros2 run turtlebot3_drl train_agent td3 0 <session_dir> <episode>
```

### Training architecture

**World:** `drl_stage4.sdf` — 4.2×4.2 m arena, creator's inner_walls layout (7 wall segments). SCUTTLE spawned as `"scuttle"`. Goal box pre-spawned at (-99,-99) and teleported by `drl_gazebo.py` via `gz service set_pose`.

**LiDAR:** 360 samples, full 360° (LD19 T-Mini Plus). Was 180° forward-only in old repo — key reason for switching.

**State (364-dim):** 360 LiDAR range values (normalised 0–1) + goal distance + goal angle + prev linear action + prev angular action.

**Actions (2-dim):** linear velocity `[0, 0.5]` m/s (SCUTTLE max), angular velocity `[-1, 1]` rad/s.

**Reset mechanism:** `drl_gazebo.py` calls `gz service /world/drl_stage4/control reset:{all:true}` on task fail. Robot and all models return to initial positions. Odom resets to zero — no frame mismatch issue.

**Models** saved every 100 episodes under `src/turtlebot3_drl/model/`.

### Key SCUTTLE adaptations vs. upstream

| File | Change |
|---|---|
| `common/settings.py` | `SPEED_LINEAR_MAX=0.5`, `SPEED_ANGULAR_MAX=1.0`, `THRESHOLD_COLLISION=0.20`, `REAL_N_SCAN_SAMPLES=360` |
| `common/utilities.py` | `get_scan_count()` returns 360; `get_simulation_speed()` returns 1 (Harmonic has no speed file) |
| `drl_gazebo/drl_gazebo.py` | Replaced `gazebo_msgs` spawn/delete/reset with `gz service` subprocess calls (Harmonic compat) |
| `scuttle_gazebo/worlds/drl_stage4.sdf` | Gazebo Harmonic world matching creator's stage4 layout |
| `scuttle_bringup/launch/drl_tomasvr.launch.py` | Launch file for drl_stage4 world + SCUTTLE spawn |

### Architecture: training nodes

| Node | Run command | Role |
|---|---|---|
| `drl_gazebo` | `ros2 run turtlebot3_drl gazebo_goals` | Spawns/hides goal, resets sim on fail |
| `drl_environment` | `ros2 run turtlebot3_drl environment` | Publishes state, computes reward, detects done |
| `drl_agent` | `ros2 run turtlebot3_drl train_agent td3` | Runs training loop, updates networks |

**Hardware deployment** (planned):
- Trained TD3/DDPG inference node publishes directly to `/cmd_vel`
- Nav2 acts as collision-avoidance safety layer only
- SLAM Toolbox provides the live occupancy grid
- LD19 LiDAR on `/scan`, micro-ROS USB motor controller on `/cmd_vel` → `/wheel_odom`

## Robot hardware reference

| Parameter | Value |
|---|---|
| Wheel radius | 0.0475 m |
| Wheel base | 0.3150 m |
| Max linear vel | 0.5 m/s |
| Max angular vel | 1.0 rad/s |
| LiDAR | LD19 T-Mini Plus, `/dev/ttyUSB1`, 230400 baud |
| Motor controller | micro-ROS USB, `/dev/ttyUSB0`, 115200 baud |

## Key topics

| Topic | Type | Direction |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR → SLAM, SAC, Nav2 |
| `/odom` | `nav_msgs/Odometry` | Motor ctrl → Nav2, SLAM |
| `/cmd_vel` | `geometry_msgs/Twist` | SAC / Nav2 → Motor ctrl |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM → Nav2, explore_lite |
| `/gdm/gas_*` | `olfaction_msgs/GasSensor` | Gas module → GDM (Phase 2) |

## GDM gas mapping (Phase 2 — placeholder)

```bash
# Log sensor + pose data
ros2 topic echo /gdm/gas_1 --csv >> ~/gas_readings.csv &
ros2 topic echo /odom --field pose.pose.position --csv >> ~/robot_positions.csv &

# Run GP-based GDM estimation
cd ~/ros2_ws/gdm
python3 gp/gp_gdm.py --readings ~/gas_readings.csv --positions ~/robot_positions.csv --output ~/gas_map.png
```
