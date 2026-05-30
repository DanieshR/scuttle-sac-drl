# SCUTTLE SAC + GDM Full Stack

## What This Is

A complete autonomous robot navigation and gas-distribution-mapping system built on a SCUTTLE differential-drive robot. The system trains a Soft Actor-Critic (SAC) deep reinforcement learning agent to navigate to goals autonomously, deploys that policy to real hardware, and integrates a real-time gas distribution mapping (GDM) pipeline with live monitoring dashboard.

**The core deliverable:** A real SCUTTLE robot that autonomously navigates to goals using a trained SAC policy, while simultaneously mapping gas concentration in the environment and streaming that data to a real-time web + RViz dashboard.

## Context

- **Stage:** Active development — simulation stack complete, training in progress (crashed, needs fix), hardware firmware not yet written, GDM pipeline integration pending
- **Urgency:** Weeks (2–4 weeks to full completion — all three pieces required)
- **Compute stack:**
  - Laptop (RTX 4060): SAC training, GDM computation, web dashboard host
  - Raspberry Pi: ROS2 runtime, SLAM Toolbox, Nav2, micro-ROS agent
  - ESP32-S3: micro-ROS firmware for motor control (cmd_vel → wheels, encoders → wheel_odom) + gas sensor bridge (Arduino MCU → ROS2 topics)
- **Gas sensors:** Physical sensors attached, bridged via Arduino/MCU to ESP32-S3

## Core Value

A SCUTTLE robot that autonomously navigates to goals using a trained SAC policy AND produces a real-time gas distribution map visible remotely — demonstrating the full sim-to-real pipeline and sensor integration.

## Requirements

### Validated

- ✓ SCUTTLE simulation in Gazebo Harmonic with ROS-GZ bridge
- ✓ SAC training loop with odom-delta position fix (goal frame mismatch resolved)
- ✓ Dense reward shaping: potential-based distance + yaw alignment
- ✓ Adaptive curriculum (target_dist ×1.01/×0.99 on success/failure)
- ✓ Headless training launch (no GUI, CPU freed)
- ✓ SLAM Toolbox online async mapping
- ✓ Nav2 safety layer (MPPI controller, collision avoidance only)
- ✓ Frontier exploration (explore_lite)
- ✓ GADEN2 gas dispersion simulation
- ✓ GP-based GDM scripts in ~/ros2_ws/gdm/ (offline)

### Active

- [ ] **DRL-01**: Training crash fixed — comms timeout no longer kills the run
- [ ] **DRL-02**: SAC converges to Phase 9 thresholds: eval/avg_goal ≥ 0.6, eval/avg_col ≤ 0.2
- [ ] **INFER-01**: SAC inference-only ROS2 node publishes /cmd_vel from saved policy weights
- [ ] **INFER-02**: Inference node uses SLAM /pose instead of odom-delta for world position
- [ ] **HW-01**: ESP32-S3 micro-ROS firmware: cmd_vel → differential drive motor commands
- [ ] **HW-02**: ESP32-S3 micro-ROS firmware: wheel encoders → /wheel_odom feedback
- [ ] **HW-03**: Gas sensor MCU bridge: sensor readings → /gdm/gas_* ROS2 topics via micro-ROS
- [ ] **HW-04**: Full hardware integration validated: RPi + ESP32-S3 + LiDAR + SAC inference running on real SCUTTLE
- [ ] **GDM-01**: GP-based GDM runs in real-time on live sensor data (not offline batch)
- [ ] **GDM-02**: RViz gas map overlay displaying concentration estimates
- [ ] **GDM-03**: Web dashboard: browser-based real-time gas map accessible over WiFi (rosbridge_server)

### Out of Scope

- Multi-robot coordination — single SCUTTLE only
- GADEN2 sim used for GDM (using real sensors, not simulated gas)
- Cloud/remote deployment of dashboard (LAN only)
- SAC training from scratch on headless 2D sim (was tried, not a clean fallback)
- Report writing — handled manually, not tracked by GSD

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Odom-delta for world position in training | Gazebo Harmonic set_pose doesn't reset odom; delta approach matches hardware SLAM math | ✓ Implemented |
| ±100 terminal rewards (not ±2500) | SAC lr=1e-4 tuned for this scale; large terminals spiked critic loss to 113k | ✓ Implemented |
| Headless Gazebo rendering | Freed ~250% CPU from gz-sim GUI process | ✓ Implemented |
| SLAM pose replaces odom-delta on hardware | Same math, SLAM gives absolute position vs. odom accumulation | — Pending |
| rosbridge_server for web dashboard | Native ROS2 ecosystem integration, no custom backend needed | — Pending |
| ESP32-S3 handles both motors and gas sensor bridge | Single MCU reduces wiring complexity; micro-ROS supports multiple publishers | — Pending |

## Architecture

```
[Training / Laptop]
  Gazebo Harmonic ─── ROS-GZ bridge ─── SAC trainer (Python, ~/drl_env)
                                              │
                                         TensorBoard, model .pth files

[Real Robot]
  LD19 LiDAR ──────────── /scan ──────┐
  RPi (ROS2)                          ├── SLAM Toolbox → /map
    micro-ROS agent ─── USB ─── ESP32-S3
                                   ├── Motor firmware → /cmd_vel → wheels
                                   ├── Encoder firmware → /wheel_odom
                                   └── Gas bridge → /gdm/gas_1..4

  SAC inference node → /cmd_vel (reads /scan + SLAM /pose)
  Nav2 (safety layer only, MPPI) → collision avoidance wrapper

[Dashboard / Laptop over WiFi]
  rosbridge_server ─── WebSocket ─── Browser (real-time gas map)
  RViz ─────────────────────────────── Gas map overlay
  GDM node (GP) ─────────────────────── /gdm/concentration_map
```

## Evolution

This document evolves at phase transitions and milestone boundaries.

**After each phase transition** (via `/gsd-transition`):
1. Requirements invalidated? → Move to Out of Scope with reason
2. Requirements validated? → Move to Validated with phase reference
3. New requirements emerged? → Add to Active
4. Decisions to log? → Add to Key Decisions
5. "What This Is" still accurate? → Update if drifted

**After each milestone** (via `/gsd-complete-milestone`):
1. Full review of all sections
2. Core Value check — still the right priority?
3. Audit Out of Scope — reasons still valid?
4. Update Context with current state

---
*Last updated: 2026-05-30 after initialization*
