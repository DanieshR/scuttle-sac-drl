# Cross-network QoS contract (Phase 3)

QoS for the topics that cross the hotspot. Getting these wrong doesn't error — it
*silently* drops data (incompatible pub/sub QoS never connect), which looks like a
flaky link. The stock nodes already use the right profiles; this documents the
contract so it survives future edits, and flags the one trap.

| Topic | Direction | Reliability | Durability | Notes |
|---|---|---|---|---|
| `/scan` | Pi → base | **best_effort** | volatile | sensor_data profile. High rate, lossy-tolerant — never reliable (would retransmit stale scans over a congested link). |
| `/tf` | both | reliable | volatile | carries `map→odom` (base→Pi) and `odom→base_link` (local). |
| `/tf_static` | both | reliable | **transient_local** | latched so late joiners get the static frames. |
| `/map` | base → Pi | reliable | **transient_local** | latched, published **on update only** (slam_toolbox does this) — never streamed. Biggest payload; transient_local lets Nav2 fetch the latest on (re)connect without waiting for the next update. |
| `navigate_to_pose` | base → Pi | reliable | volatile | action goals from frontier → Nav2. Default action QoS. |
| `/gdm/gas_1` | Pi → base | reliable | volatile | small; reliable is cheap. |
| `/estop`, `/cmd_vel*` | local to Pi | — | — | NOT cross-network. The motor loop stays on the Pi by design. |

## The one trap: /scan reliability mismatch

`/scan` is published **best_effort** (sensor_data). Any subscriber that requests
**reliable** is QoS-incompatible and receives nothing. The usual offender is Nav2's
obstacle/voxel layer `observation_sources`. Ensure the costmap scan source uses a
sensor-data / best-effort QoS (Nav2 exposes per-source QoS; the stock SCUTTLE
`nav2_params.yaml` obstacle_layer is already configured for the LD/Tmini scan).

## Pointclouds

None cross the link today (only `/scan` LaserScan). If that ever changes, **downsample
on the Pi first** (voxel filter) — never ship raw clouds over the hotspot.

## Enforcing via QoS overrides (optional)

ROS 2 nodes that support the `qos_overrides` parameter can be pinned without code
changes, e.g. for slam_toolbox's `/map` publisher:

```yaml
slam_toolbox:
  ros__parameters:
    qos_overrides:
      /map:
        publisher:
          reliability: reliable
          durability: transient_local
          history: keep_last
          depth: 1
```

Only some nodes honor `qos_overrides`; verify with `ros2 topic info -v <topic>` that
the actual endpoint QoS matches this table after bring-up.
