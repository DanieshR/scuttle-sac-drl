# `deploy/` — distributed ROS 2 offload (SCUTTLE)

Moves the non-real-time workloads — **SLAM, frontier exploration, GDM** — off the
Raspberry Pi 5 and onto a base-station laptop, while everything that must survive a
flaky link — **sensors, motor/odom, Nav2, the safety watchdog** — stays on the Pi.
The two machines talk over `rmw_zenoh_cpp`, time-synced with chrony.

This directory is **additive**: it does not modify the upstream SCUTTLE packages. It
composes the robot's own proven launch pieces into a split-across-two-machines
deployment.

## Why this split is safe

`scuttle_base` publishes `odom→base_link` **locally on the Pi** (`publish_tf:True`),
so the motor control loop never depends on the network. Only `map→odom` (from SLAM)
crosses the link. If the link drops, the Pi still knows where it is relative to its
start — and the Phase 5 watchdog asserts the e-stop. See [`watchdog/`](watchdog/).

## Layout

```
deploy/
├── README.md                     ← you are here
├── HANDOFF.md                    ← operator runbook + the [HUMAN] gates  ★ read before a live run
├── launch/
│   ├── robot_bringup.launch.py        Pi side: sensors + odom + twist_mux + Nav2
│   └── basestation_bringup.launch.py  laptop side: SLAM + frontier + (optional) GDM
├── scripts/
│   ├── whoami-net.sh             print this machine's IP (see caveat below)
│   ├── zenoh-env.sh <base|pi> [ip]   SOURCE to put a shell on zenoh
│   ├── start-router.sh           start the single zenoh router (base station)
│   ├── apply-chrony.sh <base|pi> [ip]   install chrony role config (sudo)
│   └── apply-sysctl.sh           install 16 MB UDP buffers (sudo)
├── config/
│   ├── chrony/                   base-station server + Pi client configs
│   ├── sysctl/                   99-ros2-udp-buffers.conf
│   ├── zenoh/                    topology + Fast DDS fallback notes
│   └── qos/cross_network_qos.md  cross-link QoS contract + /scan trap
├── watchdog/link_watchdog.py     link-loss → /estop lock (run as a local daemon on Pi)
└── baseline/
    ├── README.md  + pi/**        Phase 0 snapshot of the robot-side stack
    └── live/                     Phase 4 verified results (this deployment)
```

## The link

Verified over a **direct wired link** (recommended for bring-up):

| | IP (`enp5s0`) |
|---|---|
| Base station (laptop) | `192.168.155.145` |
| Pi (`amr@amr-pi`) | `192.168.155.10` |

A phone-hotspot path (`10.118.109.x`) also works; chrony/zenoh configs allow both
subnets. The wired link measured **0.118 ms RTT, 941 Mbit/s, 13 ns clock offset** —
effectively a perfect LAN.

> ⚠️ **`whoami-net.sh` caveat:** it prints the *default-route* source IP, which on a
> laptop with WiFi up is the **WiFi** address, not the wired `enp5s0`. For the wired
> deployment get the base-station IP with `ip -4 addr show enp5s0` (→ `192.168.155.145`)
> and pass that to `zenoh-env.sh pi <ip>` / `apply-chrony.sh pi <ip>`.

## Quick start (see HANDOFF.md for the full runbook)

**One-time, both machines (sudo):** install `ros-jazzy-rmw-zenoh-cpp` + `chrony`;
apply chrony + sysctl (`apply-chrony.sh`, `apply-sysctl.sh`); confirm
`chronyc tracking` on the Pi shows it locked to the base station.

**Every session:**
1. Base: `source deploy/scripts/zenoh-env.sh base` then `./deploy/scripts/start-router.sh` (leave running).
2. Pi: `source deploy/scripts/zenoh-env.sh pi 192.168.155.145` in every robot shell.
3. Pi: `ros2 launch deploy/launch/robot_bringup.launch.py`
4. Base: `ros2 launch deploy/launch/basestation_bringup.launch.py`
5. Pi (separate shell): run the watchdog — `python3 deploy/watchdog/link_watchdog.py`

## Status

Verified stationary (robot not moving): full distributed SLAM round-trip over the
wired zenoh link — see [`baseline/live/README.md`](baseline/live/README.md).
**Not yet exercised live:** Nav2 autostart under zenoh, frontier driving, and the
watchdog cable-pull e-stop test — these require an operator at the robot. The
checklist is in [HANDOFF.md](HANDOFF.md).
