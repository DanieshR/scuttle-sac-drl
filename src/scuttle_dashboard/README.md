# scuttle_dashboard

Off-board **web operator console** for the SCUTTLE gas-detection AMR. It's the integrated
cockpit prototype (`.planning/sketches/app/`) turned into a real ROS 2 package: a single-page
HMI served over HTTP and bridged to the live ROS 2 graph via **rosbridge**.

It runs on the **base-station laptop** (alongside SLAM / frontier / GDM / zenoh router) and, because
the laptop is zenoh-sourced, also sees the **Pi's** topics (Nav2, `/odom`, `/scan`, `/estop`) over the link.

## Quick start

```bash
sudo apt install ros-jazzy-rosbridge-suite          # one-time: the websocket bridge
# one-time: vendor roslib (see web/vendor/README.md)
( cd src/scuttle_dashboard/web/vendor && curl -L -o roslib.min.js https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js )

cd ~/ros2_ws
colcon build --symlink-install --packages-select scuttle_dashboard
source install/setup.bash

ros2 launch scuttle_dashboard dashboard.launch.py   # then open http://localhost:8080
```

Bring up the rest of the stack as usual (`basestation_bringup.launch.py` on the laptop,
`robot_bringup.launch.py` on the Pi). The console attaches read-only and never publishes
`/cmd_vel_teleop` until you click **Enter cockpit** and switch to MANUAL.

## How it works

- **`web/index.html`** — the UI + a simulation engine. Untouched render layer (`draw()`,
  `updateHUD()`, the `S` state object).
- **`web/ros-bindings.js`** — the live data layer. On load it tries `ws://localhost:9090`:
  - **reachable** → `S.live = true`, the sim is frozen, every panel is fed from ROS 2.
  - **not reachable / roslib missing** → silently stays in **SIM** mode (top-right badge) so the
    page still opens for review.
- **`launch/dashboard.launch.py`** — starts `rosbridge_websocket` + `rosapi` and a static HTTP
  server for `web/`.

## ROS 2 interface (see `.planning/sketches/app/DATA-CONTRACT.md` for the full map)

| Panel | Topic / action | Type | Dir |
|---|---|---|---|
| Gas hero / trend / heatmap | `/gdm/gas_1` | `olfaction_msgs/GasSensor` (0–110, **fake_gas_sensor placeholder**) | sub |
| Gas heatmap (grid) | `/gdm/gmrf_gas_map` *(confirm name)* | `nav_msgs/OccupancyGrid` | sub |
| Map | `/map` | `nav_msgs/OccupancyGrid` | sub |
| Robot pose / trajectory | `/tf` + `/tf_static` (composed `map→base_link`) | `tf2_msgs/TFMessage` | sub |
| Velocity readout | `/odom` | `nav_msgs/Odometry` | sub |
| Lidar overlay | `/scan` | `sensor_msgs/LaserScan` | sub |
| Diagnostics drawer | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | sub |
| Teleop | `/cmd_vel_teleop` (clamped 0.5 / 1.0) | `geometry_msgs/Twist` | **pub** |
| E-STOP | `/estop` (latched, twist_mux lock pri 255) | `std_msgs/Bool` | pub + sub |
| Goals | `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | action |

## Known TODOs before live use

1. **Confirm the GMRF output topic name** — `ros-bindings.js` subscribes to `/gdm/gmrf_gas_map`;
   verify against `gmrf_node` (`ros2 topic list | grep gmrf`) and update if different.
2. **Gas thresholds** — caution 60 / hazard 90 on the 0–110 GMRF scale are placeholders tied to
   `fake_gas_sensor`. Re-tune to the real sensor's units/LEL when the module lands.
3. **Frontier status** — the autonomy panel currently infers exploring/idle from goal activity.
   If you want explicit state + a real coverage %, add a small publisher; otherwise the Nav2-goal
   derivation stands (no robot-side change).
4. **`/diagnostics` keys** — the bindings expect `cpu.load/temp_c`, `battery.voltage/percentage`,
   `motor.current_left/right`, `wifi.rssi/rtt_ms`. Adjust to whatever the Pi actually publishes.
