# DRL Navigation — SCUTTLE Robot

## Project

SAC-based deep reinforcement learning for autonomous navigation on a physical SCUTTLE robot.
Forked from [tomasvr/turtlebot3_drlnav](https://github.com/tomasvr/turtlebot3_drlnav) (originally reiniscimurs/DRL-Robot-Navigation-ROS2).

**Why this fork:** TurtleBot3 had a 180° front-facing LiDAR; SCUTTLE has 360°. State dimension mismatch made the old pretrain data invalid and training unstable.

## Stack

- ROS2 Humble, Gazebo Classic (gazebo11), Docker on Linux
- Docker image: `drl_docker-drl_training:latest` (already built)
- Workspace mounts as volume to `/home/turtlebot3_drlnav` inside container
- Multi-node: `drl_gazebo` + `drl_environment` + `drl_agent` as separate ROS2 nodes

## Robot: SCUTTLE

- Differential drive: wheel_sep=0.4247m, wheel_dia=0.0833m
- YDLidar X4: 40 samples, 0–6.28 rad (full 360°), range 0.12–3.5m
- LiDAR link named `base_scan` (required by `get_scan_count()`)
- Model SDF: `src/turtlebot3_simulations/turtlebot3_gazebo/models/scuttle/model.sdf`
- URDF package: `scuttle_classic/` (package name: `scuttle_description`) — must be in `src/` to build
- All 10 DRL stage launch files hardcoded to `TURTLEBOT3_MODEL='scuttle'`
- World files: `src/.../worlds/turtlebot3_drl_stage*/scuttle.model`

## Algorithm: SAC

- Implementation: `src/turtlebot3_drl/turtlebot3_drl/drl_agent/sac.py`
- Settings: `src/turtlebot3_drl/turtlebot3_drl/common/settings.py`
  - `SAC_ALPHA=0.2`, `SAC_ALPHA_LR=3e-4`, `LEARNABLE_TEMPERATURE=True`
  - `OBSERVE_STEPS=25000` (random exploration before SAC updates begin)
  - `EPISODE_TIMEOUT_SECONDS=50`
  - `SPEED_LINEAR_MAX=0.22 m/s`, `SPEED_ANGULAR_MAX=2.0 rad/s` (may need tuning for SCUTTLE)
  - `THRESHOLD_COLLISION=0.13m`, `THREHSOLD_GOAL=0.20m`
- Registered in `drl_agent.py` alongside dqn/ddpg/td3
- Train command (inside container): `ros2 run turtlebot3_drl train_agent sac`

## Progressive Training

`progressive_train.py` — runs stages 1–9 sequentially with curriculum transfer.
- Advances when 70% success rate over last 20 episodes, 3 consecutive checks
- Max 600 episodes per stage before forced advance
- Copies weights + replay buffer between stages
- Resume: `python3 progressive_train.py --start-stage N`
- State persisted at: `output/progressive_state.json`

## Running

```bash
# Full progressive training (stages 1–9)
docker compose run --rm drl_training python3 /home/turtlebot3_drlnav/progressive_train.py

# Single stage
docker compose run --rm drl_training python3 /home/turtlebot3_drlnav/progressive_train.py --start-stage 1 --end-stage 1
```

Entrypoint (`entrypoint.sh`) auto-builds workspace on first run if `install/` is absent.

## Known Issues / Open Items

- `scuttle_description` package (`scuttle_classic/`) must be symlinked into `src/` for the URDF to build — `robot_state_publisher.launch.py` requires it to spawn the robot into Gazebo
- Obstacle plugin is a precompiled C++ `.so` — must match Humble/Ubuntu 22.04 ABI (may need recompile)
- `SPEED_LINEAR_MAX` / `SPEED_ANGULAR_MAX` not yet tuned for SCUTTLE physical limits

## Working Preferences

- Concise answers — lead with the fix, reasoning after if needed
- Frame fixes as "restart with this change", not patching a running container
- When presenting options, always end with a clear recommendation
- Monitoring: silent by default, alert only on meaningful events (training runs overnight)
