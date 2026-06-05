# SCUTTLE Navigation

Navigation stack for SCUTTLE robot including SLAM Toolbox and Nav2.

## Usage

### SLAM Mapping
```bash
# Terminal 1: Launch simulation
ros2 launch scuttle_bringup sim.launch.py

# Terminal 2: Launch SLAM
ros2 launch scuttle_nav slam.launch.py

# Terminal 3: Drive with teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4: Visualize in RViz
rviz2
```

### Save Map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/scuttle_nav/maps/my_map
```

## Configuration

- `config/slam_toolbox_params.yaml` - SLAM Toolbox parameters
- `maps/` - Saved maps directory
