# SCUTTLE SAC + GDM Implementation Plan
> ROS2 Jazzy | Gazebo Harmonic | Ubuntu 24.04

---

## Overview

**Goal:** Full room coverage exploration using SAC-based DRL on a SCUTTLE differential drive robot, with a GDM-ready pipeline for a gas sensor module.

**Stack:**
- Navigation: Nav2 + SLAM Toolbox
- Coverage: `m-explore-ros2` (frontier exploration)
- DRL Local Planner: `DRL-Robot-Navigation-ROS2` (SAC/TD3)
- Gas Simulation: `gaden2`
- Gas Distribution Mapping: `andresgongora/gdm`

---

## Phase 0 — System Prerequisites

### 0.1 Install ROS2 Jazzy

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repo
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools python3-colcon-common-extensions

# Source (add to ~/.bashrc)
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 0.2 Install Gazebo Harmonic

```bash
sudo apt install -y gz-harmonic
```

### 0.3 Install Nav2 and SLAM Toolbox

```bash
sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-amcl \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-robot-localization
```

### 0.4 Install Python dependencies

```bash
sudo apt install -y python3-pip python3-rosdep2
pip3 install torch torchvision numpy scipy
rosdep update
```

---

## Phase 1 — Workspace Setup

```bash
mkdir -p ~/scuttle_ws/src
cd ~/scuttle_ws

# Init rosdep
sudo rosdep init || true
rosdep update
```

---

## Phase 2 — SCUTTLE ROS2 Driver

```bash
cd ~/scuttle_ws/src

# Clone SCUTTLE ROS2 packages
git clone https://github.com/scuttlerobot/scuttle_ros2.git

# Install dependencies
cd ~/scuttle_ws
rosdep install -i --from-path src --rosdistro jazzy -y

# Build
colcon build --symlink-install
source install/setup.bash
```

> **Verify topics are live on the real robot:**
> ```bash
> ros2 topic list
> # Expect: /scan, /odom, /cmd_vel, /tf, /tf_static
> ros2 topic echo /scan --once
> ros2 topic echo /odom --once
> ```

---

## Phase 3 — SLAM Toolbox (Live Mapping)

```bash
# Launch SLAM in mapping mode (online async)
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=src/scuttle_ros2/config/slam_toolbox_params.yaml \
  use_sim_time:=false
```

> **Monitor map in RViz2:**
> ```bash
> rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
> ```
> Add topic `/map` (OccupancyGrid) to confirm SLAM is building.

---

## Phase 4 — Nav2 Bringup

```bash
# Terminal 1 — Nav2 stack (no pre-built map, SLAM provides it live)
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  params_file:=src/scuttle_ros2/config/nav2_params.yaml

# Terminal 2 — Verify Nav2 is up
ros2 lifecycle list /bt_navigator
ros2 service list | grep nav2
```

> **Key nav2_params.yaml values to confirm for SCUTTLE:**
> ```yaml
> robot_radius: 0.20          # SCUTTLE ~20cm radius
> max_vel_x: 0.3
> max_vel_theta: 1.0
> cmd_vel_topic: /cmd_vel     # SCUTTLE default
> odom_topic: /odom
> scan_topic: /scan
> ```

---

## Phase 5 — Frontier Exploration (Full Coverage)

### 5.1 Install m-explore-ros2

```bash
cd ~/scuttle_ws/src
git clone https://github.com/robo-friends/m-explore-ros2.git

cd ~/scuttle_ws
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --symlink-install --packages-select explore_lite
source install/setup.bash
```

### 5.2 Configure exploration params

Edit `m-explore-ros2/explore/config/params.yaml`:

```yaml
exploration_costmap:
  robot_radius: 0.20
  resolution: 0.05

explore:
  robot_base_frame: base_link
  costmap_topic: /map
  costmap_updates_topic: /map_updates
  visualize: true
  planner_frequency: 0.33
  progress_timeout: 30.0
  potential_scale: 3.0
  gain_scale: 1.0
  transform_tolerance: 0.3
  min_frontier_size: 0.75
```

### 5.3 Launch exploration

```bash
# Terminal 3 — Frontier exploration
ros2 launch explore_lite explore.launch.py \
  params_file:=src/m-explore-ros2/explore/config/params.yaml

# Monitor frontiers
ros2 topic echo /explore/frontiers

# Stop / Resume exploration
ros2 topic pub /explore/resume std_msgs/Bool "data: false" --once  # stop
ros2 topic pub /explore/resume std_msgs/Bool "data: true"  --once  # resume

# Save map once coverage is complete
ros2 run nav2_map_server map_saver_cli -f ~/maps/room_map
```

---

## Phase 6 — SAC DRL Local Planner

### 6.1 Clone and build

```bash
cd ~/scuttle_ws/src
git clone https://github.com/reiniscimurs/DRL-Robot-Navigation-ROS2.git

cd ~/scuttle_ws
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --symlink-install --packages-select drl_navigation_ros2
source install/setup.bash
```

### 6.2 Remap topics for SCUTTLE

In `DRL-Robot-Navigation-ROS2/src/drl_navigation_ros2/train.py`, confirm or patch:

```python
# These should already match SCUTTLE defaults
LASER_TOPIC  = '/scan'
ODOM_TOPIC   = '/odom'
CMD_VEL_TOPIC = '/cmd_vel'
```

### 6.3 Train SAC in Gazebo simulation first

```bash
# Terminal 1 — Gazebo simulation world
ros2 launch drl_navigation_ros2 ros2_drl.launch.py

# Terminal 2 — Start SAC training
cd ~/scuttle_ws
python3 src/DRL-Robot-Navigation-ROS2/src/drl_navigation_ros2/train.py \
  --algorithm SAC \
  --max_episodes 2000 \
  --save_interval 100
```

> Checkpoints saved to `./models/`. Monitor training loss:
> ```bash
> tensorboard --logdir ./logs/
> ```

### 6.4 Test trained policy

```bash
python3 src/DRL-Robot-Navigation-ROS2/src/drl_navigation_ros2/test.py \
  --algorithm SAC \
  --model_path ./models/SAC_best.pth
```

### 6.5 Deploy on real SCUTTLE

```bash
# Copy trained model to robot (if training on a separate machine)
scp ./models/SAC_best.pth pi@<scuttle_ip>:~/scuttle_ws/models/

# On SCUTTLE — run inference node
python3 src/DRL-Robot-Navigation-ROS2/src/drl_navigation_ros2/test.py \
  --algorithm SAC \
  --model_path ~/scuttle_ws/models/SAC_best.pth \
  --use_sim_time false
```

---

## Phase 7 — GADEN2 Gas Simulation (Pre-hardware Validation)

### 7.1 Install GADEN2

```bash
cd ~/scuttle_ws/src
git clone https://github.com/BAMresearch/gaden2.git

# GADEN depends on olfaction_msgs
git clone https://github.com/MAPIRlab/olfaction_msgs.git

cd ~/scuttle_ws
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --symlink-install --packages-select gaden2 olfaction_msgs
source install/setup.bash
```

### 7.2 Run a gas dispersion scenario

```bash
# Launch pre-built office scenario (included in gaden2/scenarios/)
ros2 launch gaden2 gaden_player.launch.py \
  scenario:=office \
  use_sim_time:=true

# Visualize gas plume in RViz2
rviz2
# Add: MarkerArray topic → /gaden2/gas_concentration_markers
```

### 7.3 Simulated gas sensor node

```bash
# Publishes synthetic sensor readings on /gaden2/sensor/reading
ros2 run gaden2 simulated_gas_sensor \
  --ros-args \
  -p sensor_model:=MOX \
  -p update_rate:=1.0
```

---

## Phase 8 — Gas Distribution Mapping (GDM)

### 8.1 Install GDM

```bash
cd ~/scuttle_ws/src
git clone https://github.com/andresgongora/gdm.git

pip3 install numpy scipy matplotlib scikit-learn
```

### 8.2 Wire gas sensor readings into GDM

The GDM scripts expect CSV input of `[x, y, concentration]` readings. Create a ROS2 subscriber bridge:

```bash
# Log sensor readings from GADEN2 or real hardware to CSV
ros2 topic echo /gaden2/sensor/reading \
  --field concentration \
  --csv > ~/gas_readings.csv &

# Simultaneously log robot pose
ros2 topic echo /odom \
  --field pose.pose.position \
  --csv > ~/robot_positions.csv &
```

### 8.3 Run GDM estimation

```bash
cd ~/scuttle_ws/src/gdm

# Example: Gaussian Process GDM
python3 gp/gp_gdm.py \
  --readings ~/gas_readings.csv \
  --positions ~/robot_positions.csv \
  --output ~/gas_map.png
```

---

## Phase 9 — Full System Launch (All-in-One)

Once all phases are validated individually, launch in this terminal order:

```bash
# T1 — SCUTTLE hardware drivers
ros2 launch scuttle_ros2 scuttle_bringup.launch.py

# T2 — SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false

# T3 — Nav2
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false \
  params_file:=~/scuttle_ws/src/scuttle_ros2/config/nav2_params.yaml

# T4 — Frontier coverage exploration
ros2 launch explore_lite explore.launch.py

# T5 — SAC inference node
python3 ~/scuttle_ws/src/DRL-Robot-Navigation-ROS2/src/drl_navigation_ros2/test.py \
  --algorithm SAC \
  --model_path ~/scuttle_ws/models/SAC_best.pth

# T6 — Gas sensor node (real hardware or GADEN2 sim)
ros2 run your_gas_sensor_pkg gas_sensor_node   # replace with your sensor driver

# T7 — GDM logging bridge
ros2 topic echo /gas_sensor/reading --csv >> ~/gas_readings.csv
```

---

## Topic Interface Reference

| Topic | Type | Producer | Consumer |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | SCUTTLE LiDAR | SLAM, SAC, Nav2 |
| `/odom` | `nav_msgs/Odometry` | SCUTTLE driver | Nav2, SLAM, GDM |
| `/cmd_vel` | `geometry_msgs/Twist` | SAC / Nav2 | SCUTTLE driver |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox | Nav2, m-explore |
| `/explore/frontiers` | `visualization_msgs/MarkerArray` | m-explore | RViz2 |
| `/gas_sensor/reading` | `olfaction_msgs/GasSensor` | Gas module / GADEN2 | GDM |

---

## Troubleshooting

**Nav2 not activating lifecycle nodes:**
```bash
ros2 lifecycle set /bt_navigator activate
ros2 lifecycle set /planner_server activate
ros2 lifecycle set /controller_server activate
```

**SLAM map not publishing:**
```bash
ros2 topic hz /map        # should be ~1Hz
ros2 topic hz /scan       # should match LiDAR rate (e.g. 10Hz)
```

**SAC node can't find `/scan`:**
```bash
# Remap at launch
ros2 run drl_navigation_ros2 train.py \
  --ros-args -r /scan:=/your_actual_scan_topic
```

**colcon build fails on Jazzy (API changes from Foxy):**
```bash
# Check for deprecated rclpy/rclcpp APIs
grep -r "rclpy.spin_once" src/DRL-Robot-Navigation-ROS2/
# Replace with executor pattern if needed
```

---

## Key Repos

| Repo | URL |
|---|---|
| DRL-Robot-Navigation-ROS2 | https://github.com/reiniscimurs/DRL-Robot-Navigation-ROS2 |
| m-explore-ros2 | https://github.com/robo-friends/m-explore-ros2 |
| GADEN2 | https://github.com/BAMresearch/gaden2 |
| GADEN (MAPIRlab) | https://github.com/MAPIRlab/gaden |
| GDM scripts | https://github.com/andresgongora/gdm |
| olfaction_msgs | https://github.com/MAPIRlab/olfaction_msgs |
| Nav2 (Jazzy branch) | https://github.com/ros-navigation/navigation2/tree/jazzy |
| SLAM Toolbox | https://github.com/SteveMacenski/slam_toolbox |
| CTSAC paper (S2R upgrade) | https://arxiv.org/abs/2503.14254 |
