# How to run the mapping trials (Sections 3.2 / 3.3)

Run 5 trials per method. Each trial = one terminal per command below.

---

## Trial setup (same for every method)

**Terminal 1 — Gazebo + SLAM:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch scuttle_bringup sim.launch.py world:=maze   # or your exploration world
# (also starts slam_toolbox or cartographer if wired into sim.launch.py)
```

**Terminal 2 — Frontier detector** (you need a frontier_exploration node):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run frontier_exploration frontier_explorer
```

---

## Method A — SAC+Frontier (proposed)

**Terminal 3 — SAC inference:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run turtlebot3_drl drl_inference
```

**Terminal 4 — Trial recorder:**
```bash
cd ~/Desktop/results
python3 run_mapping_trial.py --method sac_frontier --trial 1 --duration 180
```
Repeat with --trial 2 through 5, restarting the sim each time.

---

## Method B — DWA+Frontier (baseline)

Replace Terminal 3 with Nav2 DWA:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```
Make sure the frontier node publishes goals to /goal_pose (Nav2 picks them up automatically).

**Terminal 4:**
```bash
python3 run_mapping_trial.py --method dwa_frontier --trial 1 --duration 180
```

---

## Method C — Random Walk (baseline)

No navigation stack needed. The recorder publishes random cmd_vel itself.

**Terminal 3 — Trial recorder (self-contained):**
```bash
python3 run_mapping_trial.py --method random_walk --trial 1 --duration 180
```

---

## After all 15 trials (5×3 methods):

```bash
cd ~/Desktop/results
python3 analyze_mapping_results.py
```

This generates:
- fig7_coverage_over_time.png
- fig8_mapping_benchmark_bars.png
- mapping_summary.csv  ← paste numbers into Table 2 of RESULTS_SECTION.md
