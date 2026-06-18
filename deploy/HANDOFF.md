# HANDOFF — distributed offload operator runbook

Everything an operator needs to bring the split stack up on real hardware, plus the
gates that still require a human at the robot. Read this fully before a live run.

## Machines

| Role | Host | Wired IP (`enp5s0`) | User |
|---|---|---|---|
| Base station | laptop (`daniesh-Legion-Pro-5`) | `192.168.155.145` | `daniesh` |
| Robot | Raspberry Pi 5 (`amr-pi`) | `192.168.155.10` | `amr` |

SSH: `ssh amr@192.168.155.10` (key-based, no password). Hotspot path `10.118.109.x`
also works; configs allow both subnets.

> **Finding the base-station IP:** `ip -4 addr show enp5s0`. Do NOT trust
> `whoami-net.sh` on the wired link — it returns the WiFi default-route IP. (See
> README caveat.)

## One-time setup (sudo on both machines)

```bash
# both machines:
sudo apt update && sudo apt install -y ros-jazzy-rmw-zenoh-cpp chrony iperf3

# base station:
sudo ./deploy/scripts/apply-chrony.sh base
sudo ./deploy/scripts/apply-sysctl.sh

# Pi (run from a checkout of deploy/, OR copy the rendered files over):
sudo ./deploy/scripts/apply-chrony.sh pi 192.168.155.145
sudo ./deploy/scripts/apply-sysctl.sh
```

**Gate — do not proceed until this passes:** on the Pi, `chronyc tracking` shows
`Reference ID` = `C0A89B91 (192.168.155.145)` and a sub-µs offset. (Verified: ~13 ns.)

> The Pi doesn't have the `deploy/` tree checked out. Either `git` the worktree onto
> it, or `scp` the two rendered configs to `~` and `sudo install` them — that's how
> the verified run did it (`chrony-pi-ros2-offload.conf`, `99-ros2-udp-buffers.conf`).

## Per-session bring-up order

Order matters — router first, sensors before SLAM, SLAM before Nav2.

1. **Base — zenoh router** (leave running in its own shell):
   ```bash
   source deploy/scripts/zenoh-env.sh base
   ./deploy/scripts/start-router.sh           # listens tcp/[::]:7447
   ```
2. **Pi — robot stack** (every Pi shell must source zenoh-env first):
   ```bash
   source deploy/scripts/zenoh-env.sh pi 192.168.155.145
   ros2 launch deploy/launch/robot_bringup.launch.py
   ```
3. **Base — SLAM + frontier:**
   ```bash
   source deploy/scripts/zenoh-env.sh base
   ros2 launch deploy/launch/basestation_bringup.launch.py        # enable_gdm:=false
   ```
4. **Pi — safety watchdog** (separate shell, so a bringup restart never kills it):
   ```bash
   source deploy/scripts/zenoh-env.sh pi 192.168.155.145
   python3 deploy/watchdog/link_watchdog.py --ros-args \
     -p map_frame:=map -p odom_frame:=odom
   ```

Verify cross-link health (from the base station): `ros2 topic hz /scan` (~10 Hz),
`ros2 run tf2_ros tf2_echo map base_link` resolves.

## ‼️ REQUIRED config edit before a moving run — twist_mux / collision_monitor

`robot_bringup.launch.py` **re-enables twist_mux** so the watchdog can assert the
`/estop` lock (priority 255). The robot currently ships with twist_mux OFF and Nav2's
`collision_monitor` writing straight to `/cmd_vel`. With twist_mux back on, both would
drive `/cmd_vel` and fight unless you redirect collision_monitor to the muxed input:

```
scuttle_nav/config/nav2_params.yaml  →  collision_monitor:
    cmd_vel_out_topic: "cmd_vel"   ==>   cmd_vel_out_topic: "cmd_vel_safe"
```

(twist_mux inputs: teleop `/cmd_vel_teleop` pri 100, navigation `/cmd_vel_safe` pri 10,
lock `/estop` pri 255; output remapped to `/cmd_vel`.)

> ⚠️ The Pi's `scuttle` clone is **diverged/dirty (read-only)** — back the file up
> (`cp nav2_params.yaml nav2_params.yaml.bak`) before editing, and don't commit there.

## ‼️ RISK to validate on first moving run — Nav2 autostart under zenoh

The robot's stock `exploration.launch.py` mandates **CycloneDDS** because Fast DDS
intermittently drops Nav2 lifecycle `change_state` responses and hangs autostart
(navigation2#3033). This offload uses **rmw_zenoh** — autostart reliability under
zenoh is **unverified**. On first Nav2 bring-up, watch for the lifecycle manager
hanging on "Configuring/Activating". If it hangs, fall back per
[`config/zenoh/README.md`](config/zenoh/README.md) (Fast DDS Discovery Server, or
CycloneDDS for the whole deployment).

## Deferred live tests (operator at the robot — wheels clear, e-stop reachable)

Verified stationary already (see `baseline/live/`). These still need a human present:

- [ ] Apply the `cmd_vel_safe` edit above.
- [ ] Bring up Nav2 (step 2/3) — confirm autostart completes under zenoh.
- [ ] Frontier exploration drives the robot — confirm it maps and navigates.
- [ ] **Watchdog disconnect test:** with the robot driving, pull the link (or stop the
      router). Confirm: watchdog detects stale `map→odom` → publishes `/estop` true →
      robot stops within ~1.5 s. Reconnect → after the debounce, `/estop` clears and
      control resumes.
- [ ] Re-capture live node graph + `view_frames` with the full stack (incl. Nav2).

## GDM (gas mapping)

Off by default (`enable_gdm:=false`). The live GDM nodes (`scuttle_base/map_throttle`,
`gmrf_gas_mapping/gmrf_node`) exist only on the Pi today — port them to the base
station and `colcon build` before `enable_gdm:=true`. The gas **sensor** stays on the
robot and publishes `/gdm/gas_1` across the link.

## Teardown

```bash
# stop launches with Ctrl-C in each shell, or:
pkill -f robot_bringup ; pkill -f basestation_bringup ; pkill -f rmw_zenohd

# remove the temporary passwordless-sudo grants used during setup (IMPORTANT):
sudo rm -f /etc/sudoers.d/99-claude-phase4                                  # laptop
ssh amr@192.168.155.10 'sudo rm -f /etc/sudoers.d/99-claude-phase4'         # Pi
```

## Gotchas

- **Every** Pi shell that runs ROS must `source zenoh-env.sh pi <ip>` first — the
  `ZENOH_CONFIG_OVERRIDE` is per-shell. A shell without it silently won't find the router.
- Tmini LiDAR prints checksum errors + "intensity auto-adjusted" at startup — **benign**;
  it self-corrects to a clean 10 Hz within a second.
- `/scan` is **best_effort** (sensor_data QoS). Any reliable subscriber gets nothing —
  see [`config/qos/cross_network_qos.md`](config/qos/cross_network_qos.md).
- `ros2 topic delay /scan` ≈ 100 ms is the lidar's scan period, **not** link latency.
- Laptop sleep / WiFi roaming can drop the router; the watchdog will e-stop the robot
  if `map→odom` goes stale — that's by design.
