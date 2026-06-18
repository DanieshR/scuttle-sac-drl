# Zenoh middleware (Phase 2)

`rmw_zenoh` replaces the default Fast DDS for cross-machine traffic. Plain DDS
multicast discovery is unreliable over a phone hotspot; zenoh uses a single
**router** (unicast TCP) that every node connects to, which is robust on a flaky link.

Verified against the ROS 2 **Jazzy** `rmw_zenoh` docs (github.com/ros2/rmw_zenoh).

## Install (BOTH machines)

```bash
sudo apt update && sudo apt install ros-jazzy-rmw-zenoh-cpp
```

## Topology

```
        Pi node ──(zenoh client, tcp/<base_ip>:7447)──┐
                                                       ▼
   Base node ──(zenoh, tcp/localhost:7447)──►  rmw_zenohd router  (base station)
```

One router, on the base station. Base-station nodes connect to it locally (default
config). Pi nodes connect to it across the hotspot.

## Startup order, each session

1. **Base station** — start the router (leave it running):
   ```bash
   ./deploy/scripts/start-router.sh
   ./deploy/scripts/whoami-net.sh        # note the IP -> give it to the Pi
   ```
2. **Base station** — every ROS shell here:
   ```bash
   source deploy/scripts/zenoh-env.sh base
   ```
3. **Pi** — every ROS shell there:
   ```bash
   source deploy/scripts/zenoh-env.sh pi <basestation_ip>
   ```

`zenoh-env.sh` sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, a shared `ROS_DOMAIN_ID`,
and (Pi only) `ZENOH_CONFIG_OVERRIDE` to dial the base router in **client mode**.
We override the connect endpoint at runtime rather than ship a full `*.json5`
config so we (a) never hardcode the dynamic hotspot IP and (b) don't pin a copy of
the config schema that could drift from the installed `rmw_zenoh` version.

## Verify (Phase 2 gate)

With the router up and both shells sourced, each machine should see the other's
graph across the hotspot:

```bash
ros2 node list      # shows nodes from BOTH machines
ros2 topic list     # shows topics from BOTH machines
```

## Fallback — Fast DDS Discovery Server

If zenoh proves problematic, fall back to a Fast DDS **Discovery Server** (unicast,
also avoids multicast). Do NOT fall back to plain default-multicast DDS — it is the
unreliable option this phase exists to replace. Sketch:

```bash
# Base station (server):
export ROS_DISCOVERY_SERVER=<base_ip>:11811
fastdds discovery --server-id 0 --port 11811
# Both machines export the same ROS_DISCOVERY_SERVER and unset RMW_IMPLEMENTATION
# (use default rmw_fastrtps_cpp). Super-client config gives full graph visibility.
```
