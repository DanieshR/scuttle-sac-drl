# Requirements — SCUTTLE SAC + GDM Full Stack

## v1 Requirements

### DRL Training

- [ ] **DRL-01**: Training run completes without crashing from comms timeout errors
- [ ] **DRL-02**: SAC policy achieves eval/avg_goal ≥ 0.6 across 10 evaluation scenarios
- [ ] **DRL-03**: SAC policy achieves eval/avg_col ≤ 0.2 across 10 evaluation scenarios
- [ ] **DRL-04**: Trained model weights (.pth files) are saved and verified loadable

### SAC Inference (Real Hardware)

- [ ] **INFER-01**: ROS2 inference node loads saved SAC actor weights and publishes /cmd_vel
- [ ] **INFER-02**: Inference node reads SLAM /pose (not odom-delta) for world-frame position
- [ ] **INFER-03**: Inference node reads /scan (20-beam downsampled) and computes state vector identical to training
- [ ] **INFER-04**: Inference node verified in Gazebo simulation before real-hardware deployment
- [ ] **INFER-05**: Nav2 safety layer active during inference — overrides /cmd_vel on imminent collision

### ESP32-S3 Firmware (micro-ROS)

- [ ] **HW-01**: ESP32-S3 subscribes to /cmd_vel and drives differential motors at correct wheel velocities
- [ ] **HW-02**: ESP32-S3 reads wheel encoders and publishes /wheel_odom at ≥ 20 Hz
- [ ] **HW-03**: micro-ROS agent running on RPi connects to ESP32-S3 over USB
- [ ] **HW-04**: Gas sensor module data relayed to /gdm/gas_1 (and additional channels) at ≥ 1 Hz — module is pre-calibrated, firmware is relay only
- [ ] **HW-05**: Firmware handles connection loss gracefully (watchdog, safe motor stop)

### Hardware Integration

- [ ] **INT-01**: Real SCUTTLE navigates autonomously using SAC inference + SLAM pose + LiDAR
- [ ] **INT-02**: SLAM Toolbox builds a map of the real environment in real time
- [ ] **INT-03**: Nav2 safety layer prevents collisions on real hardware
- [ ] **INT-04**: Gas sensor data streams to /gdm/gas_* topics verified with real sensor readings

### GDM Pipeline

- [ ] **GDM-01**: GP-based GDM node subscribes to /gdm/gas_* and /odom (or SLAM /pose), updates concentration estimate in real time
- [ ] **GDM-02**: GDM node publishes /gdm/concentration_map (nav_msgs/OccupancyGrid or custom msg)
- [ ] **GDM-03**: RViz displays gas concentration overlay on the SLAM map
- [ ] **GDM-04**: Web dashboard accessible over LAN (WiFi) shows real-time gas map in browser
- [ ] **GDM-05**: Web dashboard updates at ≥ 1 Hz without requiring page refresh

## v2 Requirements (Deferred)

- Multi-source gas mapping (multiple gas types / sensors)
- Autonomous gas-source seeking behaviour (integrate GDM into nav goal selection)
- Recording and playback of gas mapping sessions
- GADEN2 simulation integration with GDM pipeline
- Cloud / remote dashboard access (beyond LAN)

## Out of Scope

- Multi-robot coordination — single SCUTTLE platform only
- Report writing — tracked manually, not a GSD phase
- Headless 2D sim fallback (scuttle_sac_v2 approach) — was tried, not a clean path
- Training hyperparameter search / ablation — current proven params are used as-is

## Traceability

| Requirement | Phase | Status |
|-------------|-------|--------|
| DRL-01 | Phase 1: Stable Training | Pending |
| DRL-02 | Phase 1: Stable Training | Pending |
| DRL-03 | Phase 1: Stable Training | Pending |
| DRL-04 | Phase 1: Stable Training | Pending |
| INFER-01 | Phase 2: SAC Inference Node | Pending |
| INFER-02 | Phase 2: SAC Inference Node | Pending |
| INFER-03 | Phase 2: SAC Inference Node | Pending |
| INFER-04 | Phase 2: SAC Inference Node | Pending |
| INFER-05 | Phase 2: SAC Inference Node | Pending |
| HW-01 | Phase 3: ESP32-S3 Firmware | Pending |
| HW-02 | Phase 3: ESP32-S3 Firmware | Pending |
| HW-03 | Phase 3: ESP32-S3 Firmware | Pending |
| HW-04 | Phase 3: ESP32-S3 Firmware | Pending |
| HW-05 | Phase 3: ESP32-S3 Firmware | Pending |
| INT-01 | Phase 4: Hardware Integration | Pending |
| INT-02 | Phase 4: Hardware Integration | Pending |
| INT-03 | Phase 4: Hardware Integration | Pending |
| INT-04 | Phase 4: Hardware Integration | Pending |
| GDM-01 | Phase 5: GDM Pipeline + Dashboard | Pending |
| GDM-02 | Phase 5: GDM Pipeline + Dashboard | Pending |
| GDM-03 | Phase 5: GDM Pipeline + Dashboard | Pending |
| GDM-04 | Phase 5: GDM Pipeline + Dashboard | Pending |
| GDM-05 | Phase 5: GDM Pipeline + Dashboard | Pending |
