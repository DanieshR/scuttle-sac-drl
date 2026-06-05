# SCUTTLE SAC-DRL Autonomous Navigation & Mapping

ROS 2 (Jazzy) workspace for autonomous exploration of unknown environments using a
hybrid **Soft Actor-Critic (SAC) deep reinforcement learning** policy combined with
**frontier-based exploration** on a SCUTTLE differential-drive robot.

---

## Repository structure

```
ros2_ws/
├── src/
│   ├── scuttle_bringup/        # Top-level launch files (sim, real, DRL, frontier)
│   ├── scuttle_description/    # URDF/xacro, meshes, Gazebo Harmonic bridge config
│   ├── scuttle_gazebo/         # Worlds (.sdf), GasSourcePlugin, DRL stage worlds
│   ├── scuttle_nav/            # SLAM Toolbox + Nav2 + explore_lite launch files
│   ├── turtlebot3_drlnav/      # DRL training framework (tomasvr fork, SCUTTLE-adapted)
│   ├── m-explore-ros2/         # Frontier exploration (explore_lite)
│   ├── gaden2/                 # Gas dispersion simulation (BAMresearch)
│   └── olfaction_msgs/         # ROS2 gas sensor message types
├── drl_training/               # Docker-based SAC training environment (ROS2 Humble)
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── progressive_train.py    # 9-stage curriculum training script
│   ├── src/                    # Training source (turtlebot3_drl package)
│   └── output/                 # Per-episode training logs (.txt), figures (.png)
├── gdm/                        # Gas Distribution Mapping scripts (GP-based)
├── results/                    # Paper figures and training analysis
│   ├── fig1_reward_curve.png
│   ├── fig2_success_rate.png
│   ├── fig3_actor_critic_loss.png
│   ├── fig4_stage_success_bar.png
│   ├── fig5_stage8_zoom.png
│   ├── fig6_entropy_collapse.png
│   ├── training_summary.csv
│   └── RESULTS_SECTION.md      # Draft Results section (academic prose)
└── scuttle_sac_gdm_plan.md     # Full project roadmap
```

---

## Hardware

| Component | Spec |
|---|---|
| Robot | SCUTTLE v3.0 differential drive |
| LiDAR | LD19 T-Mini Plus, 360°, `/dev/ttyUSB1` |
| Motor controller | micro-ROS USB, `/dev/ttyUSB0` |
| Wheel separation | 0.4247 m |
| Wheel diameter | 0.0833 m |
| Max linear vel | 0.35 m/s |
| Max angular vel | 2.0 rad/s |

---

## Software stack

- **ROS 2 Jazzy** + **Gazebo Harmonic** (simulation, deployment)
- **ROS 2 Humble** + **Gazebo Classic** (SAC training, via Docker)
- **SLAM Toolbox** (online async mapping)
- **Nav2** (collision-avoidance safety layer)
- **explore_lite** (frontier exploration)
- **PyTorch** (SAC policy network)

---

## Quick start

### 1. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Simulation (Gazebo Harmonic)

```bash
# Full sim (Gazebo + robot + bridge)
ros2 launch scuttle_bringup sim.launch.py

# With SLAM mapping
ros2 launch scuttle_nav slam.launch.py

# Frontier exploration (explore_lite)
ros2 launch scuttle_nav frontier.launch.py
```

### 3. Deploy trained SAC policy (inference)

```bash
# Terminal 1 — Simulation
ros2 launch scuttle_bringup drl_tomasvr.launch.py stage:=1

# Terminal 2 — SAC inference (Stage 8 ep1300 checkpoint)
ros2 run turtlebot3_drl drl_inference

# Terminal 3 — Send a navigation goal
ros2 topic pub --once /goal_pose geometry_msgs/msg/Pose \
  '{position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}'
```

### 4. Real robot deployment

```bash
# Terminal 1
ros2 launch scuttle_bringup real.launch.py

# Terminal 2 (adds LD19 offset correction)
DRL_LIDAR_CORRECTION=0.40 ros2 run turtlebot3_drl drl_inference
```

---

## SAC training (Docker, ROS 2 Humble)

The policy was trained inside `drl_training/` using a 9-stage progressive curriculum.
See `drl_training/CLAUDE.md` for full training details.

```bash
cd ~/ros2_ws/drl_training

# Build Docker image
docker compose build

# Run full progressive training (stages 1–9)
docker compose run --rm drl_training python3 /home/turtlebot3_drlnav/progressive_train.py

# Resume from a specific stage
docker compose run --rm drl_training \
  python3 /home/turtlebot3_drlnav/progressive_train.py --start-stage 8
```

**Best deployable checkpoint:** `drl_training/src/turtlebot3_drl/model/daniesh-Legion-Pro-5-16ARX8/sac_0_stage_8/actor_stage8_episode1300.pt`

---

## Training results summary

| Stage | Episodes | Success % | Peak 20-ep SR | Actor loss (start→end) |
|---|---|---|---|---|
| 4 | 401–1049 | 56.2 | 75% | 26.9 → 16.6 |
| 5 | 1001–1121 | 67.8 | 75% | 17.9 → 15.1 |
| 6 | 1101–1218 | 71.2 | **80%** | 16.2 → 14.1 |
| 8 | 1201–1841 | 54.1 | 65% | 14.7 → 5.6 |
| 9 | 1801–2436 | 51.1 | 65% | 7.4 → **0.51** (collapse) |

Total: **2,436 episodes · ~1.22M environment steps**

See `results/` for full figures and `results/RESULTS_SECTION.md` for the paper Results section.

---

## SAC network architecture

```
State (64-dim): 60 LiDAR bins + goal_dist + goal_angle + prev_lin + prev_ang
Action (2-dim): linear velocity, angular velocity

Actor:  Linear(64→512) → ReLU → Linear(512→512) → ReLU → mean/log_std(512→2) → tanh
Critic: Twin Q-networks, each (64+2)→512→512→1
```

**Hyperparameters:** γ=0.99, τ=0.003, lr=3×10⁻³, α_lr=3×10⁻⁴, batch=128, buffer=10⁶,
target entropy=−2.0 nats, frame skip=4, AdamW.
