# Roadmap: SCUTTLE SAC + GDM Full Stack

## Overview

Starting from a working Gazebo simulation, this roadmap delivers a real SCUTTLE robot
that navigates autonomously using a trained SAC policy and simultaneously maps gas
concentrations in real time. The five phases follow the natural dependency chain:
get a converged policy (Phase 1), build the inference node that runs it (Phase 2),
write the ESP32-S3 firmware that drives the real hardware (Phase 3), integrate and
validate everything on the physical robot (Phase 4), and close out with the GDM
pipeline and LAN-accessible web dashboard (Phase 5).

## Phases

**Phase Numbering:**
- Integer phases (1, 2, 3): Planned milestone work
- Decimal phases (2.1, 2.2): Urgent insertions (marked with INSERTED)

- [ ] **Phase 1: Stable Training** - Fix comms timeout crash, reach convergence thresholds, save verified weights
- [ ] **Phase 2: SAC Inference Node** - Build and sim-validate ROS2 inference node using SLAM pose
- [ ] **Phase 3: ESP32-S3 Firmware** - micro-ROS firmware for motors, odometry, and gas sensor bridge
- [ ] **Phase 4: Hardware Integration** - Full real-robot bring-up: SAC + SLAM + Nav2 + gas sensors validated
- [ ] **Phase 5: GDM Pipeline + Dashboard** - Real-time GP gas mapping, RViz overlay, LAN web dashboard

## Phase Details

### Phase 1: Stable Training
**Goal**: A converged SAC policy exists — training completes without crashing and achieves navigation thresholds
**Depends on**: Nothing (first phase)
**Requirements**: DRL-01, DRL-02, DRL-03, DRL-04
**Success Criteria** (what must be TRUE):
  1. Training runs to completion (100 epochs × 70 episodes) without a comms timeout crash
  2. eval/avg_goal >= 0.6 across 10 evaluation scenarios (confirmed by monitor.py output)
  3. eval/avg_col <= 0.2 across 10 evaluation scenarios (confirmed by monitor.py output)
  4. SAC_actor.pth, SAC_critic.pth, SAC_critic_target.pth load without error in a fresh Python session
**Plans**: TBD

### Phase 2: SAC Inference Node
**Goal**: A ROS2 node exists that loads saved weights and drives the robot using SLAM pose — verified in Gazebo before touching hardware
**Depends on**: Phase 1
**Requirements**: INFER-01, INFER-02, INFER-03, INFER-04, INFER-05
**Success Criteria** (what must be TRUE):
  1. Inference node publishes /cmd_vel at the correct rate when /scan and SLAM /pose are available
  2. State vector computed by inference node is byte-for-byte identical to training state vector (verified by logging both)
  3. Robot navigates to a manually specified goal in Gazebo simulation using inference node alone (no training loop running)
  4. Nav2 safety layer overrides /cmd_vel and stops the robot when a simulated obstacle is placed directly in its path
**Plans**: TBD

### Phase 3: ESP32-S3 Firmware
**Goal**: The ESP32-S3 handles all real-robot I/O — motors driven from /cmd_vel, odometry published, gas sensor data bridged — with safe failure handling
**Depends on**: Phase 1
**Requirements**: HW-01, HW-02, HW-03, HW-04, HW-05
**Success Criteria** (what must be TRUE):
  1. Robot wheels respond correctly to /cmd_vel commands (manual teleop drives straight and turns as expected)
  2. /wheel_odom publishes at >= 20 Hz with correct distance and heading increments over a measured 1-metre run
  3. micro-ROS agent on RPi connects to ESP32-S3 and all topics appear in ros2 topic list
  4. /gdm/gas_1 (and additional channels) publish real sensor readings at >= 1 Hz
  5. Cutting USB power to ESP32-S3 causes motors to stop within the watchdog timeout (no runaway)
**Plans**: TBD

### Phase 4: Hardware Integration
**Goal**: The real SCUTTLE navigates a room autonomously using SAC + SLAM + Nav2, while gas sensor data streams live — all subsystems verified together on real hardware
**Depends on**: Phase 2, Phase 3
**Requirements**: INT-01, INT-02, INT-03, INT-04
**Success Criteria** (what must be TRUE):
  1. SCUTTLE navigates from a start pose to a manually set goal pose in a real room without human intervention
  2. SLAM Toolbox builds a recognisable map of the test environment during a full navigation run
  3. Nav2 safety layer halts or reroutes the robot when a person steps into its path
  4. ros2 topic echo /gdm/gas_1 shows changing real sensor values as the robot moves through different zones
**Plans**: TBD

### Phase 5: GDM Pipeline + Dashboard
**Goal**: Gas concentration estimates update in real time during navigation, visible in both RViz and a LAN browser tab — closing the full demo loop
**Depends on**: Phase 4
**Requirements**: GDM-01, GDM-02, GDM-03, GDM-04, GDM-05
**Success Criteria** (what must be TRUE):
  1. GP-based GDM node updates its concentration estimate within 2 seconds of each new /gdm/gas_* reading
  2. RViz shows a colour-coded gas concentration overlay on the SLAM map that changes visibly as the robot moves
  3. A browser on a separate LAN device loads the dashboard without installing ROS and sees the gas map update in real time
  4. Dashboard gas map refreshes at >= 1 Hz without requiring a manual page reload
**UI hint**: yes
**Plans**: TBD

## Progress

**Execution Order:**
Phases execute in numeric order: 1 → 2 → 3 → 4 → 5
(Phases 2 and 3 have no dependency on each other and may proceed in parallel)

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. Stable Training | 0/TBD | Not started | - |
| 2. SAC Inference Node | 0/TBD | Not started | - |
| 3. ESP32-S3 Firmware | 0/TBD | Not started | - |
| 4. Hardware Integration | 0/TBD | Not started | - |
| 5. GDM Pipeline + Dashboard | 0/TBD | Not started | - |
