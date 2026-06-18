# Phase 4 — live distributed bring-up results (stationary)

Captured 2026-06-18 over the **direct wired link** (Pi `192.168.155.10` ↔ laptop
`192.168.155.145`, `enp5s0`), `rmw_zenoh_cpp`, `ROS_DOMAIN_ID=0`, real clock.

This is the **stationary** half of Phase 4 — sensors + distributed SLAM with the
robot NOT moving (no Nav2 goals, no frontier). The robot-driving tests (frontier
exploration + watchdog cable-pull e-stop) are deferred to an operator-present
session.

## What was verified

| Check | Result |
|---|---|
| Installs (zenoh 0.2.9, chrony 4.5, iperf3) both machines | ✅ |
| sysctl 16 MB UDP buffers both machines | ✅ |
| chrony time sync | ✅ Pi locked to laptop (RefID 192.168.155.145), stratum 4, **offset ~13 ns** |
| zenoh router (laptop, tcp/[::]:7447) + cross-machine discovery | ✅ Pi client saw laptop nodes/topics, echoed live data |
| **`/scan` Pi → laptop** | ✅ stable **10.01 Hz**, std dev 0.5 ms |
| **`odom→base_link` Pi → laptop** | ✅ resolves via tf2_echo (19.8 Hz) |
| **SLAM on laptop** (slam_toolbox async, use_sim_time:=false) | ✅ Configuring→Activating, sensor registered |
| **`map→odom` laptop → Pi** | ✅ resolves on the Pi (10.2 Hz) |
| **`/map` laptop → Pi** | ✅ OccupancyGrid 119×165 @ 0.05 m on the Pi |
| Full TF tree coherent across link | ✅ map→odom→base_link→chassis_link→lidar_1 (see frames PDF) |

The round-trip TF contract holds: `odom→base_link` stays local to the Pi
(`publish_tf:True`), only `map→odom` crosses from SLAM — exactly the property the
offload depends on.

## Latency / bandwidth baseline (wired link)

- **ping RTT:** 0.118 ms avg, 0% loss / 50 pkts, mdev 0.017 ms
- **iperf3:** 941 Mbit/s, 0 retransmits
- **clock offset:** ~13 ns (chrony, Pi↔laptop)
- **`/scan` header→receipt:** 0.100 s — this is the Tmini's 10 Hz scan-accumulation
  period, NOT link latency (see `scan_delay.txt`).

Conclusion: the wired link is effectively a perfect LAN; the distributed offload
adds negligible latency over a single-machine setup.

## Files

- `node_graph.txt` — live `ros2 node list` + `ros2 topic list -t`
- `frames_*.pdf` / `frames_*.gv` — TF tree (view_frames)
- `scan_delay.txt` — `/scan` delay measurement + interpretation

## NOT yet done (operator-present session)

- `nav2_params.yaml` `cmd_vel_out_topic: cmd_vel → cmd_vel_safe` edit on the Pi
  (required when twist_mux is re-enabled — see robot_bringup header).
- Nav2 bring-up + **autostart-under-zenoh** verification (the documented risk vs
  CycloneDDS / navigation2#3033).
- Frontier exploration (robot drives).
- Watchdog disconnect test: pull cable → `/estop` engages → robot stops → recovers.
