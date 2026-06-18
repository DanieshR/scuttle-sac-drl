# zenoh-env.sh — SOURCE this (don't execute) to put a shell on the zenoh middleware.
#
#   On the base station:   source deploy/scripts/zenoh-env.sh base
#   On the Pi:             source deploy/scripts/zenoh-env.sh pi <basestation_ip>
#
# Get <basestation_ip> by running deploy/scripts/whoami-net.sh ON THE BASE STATION.
# Because the hotspot lease changes every session, the IP is passed in at runtime —
# never baked into a file.
#
# What it sets:
#   RMW_IMPLEMENTATION=rmw_zenoh_cpp   (both machines must match)
#   ROS_DOMAIN_ID                       (shared; defaults to 0)
#   ZENOH_CONFIG_OVERRIDE               (Pi only) -> connect to the base station's
#                                       router in client mode. Client mode avoids
#                                       relying on multicast peer discovery, which
#                                       is unreliable over a phone hotspot.
#
# NOTE: sourced script — no `set -e`/`exit`, or it would kill your interactive shell.

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

_zenoh_role="${1:-}"
case "${_zenoh_role}" in
  base)
    # Base station nodes use the default session config: they find the local
    # router at tcp/localhost:7447 automatically. Nothing else to set.
    unset ZENOH_CONFIG_OVERRIDE
    echo "[zenoh-env] base: RMW=rmw_zenoh_cpp DOMAIN=${ROS_DOMAIN_ID} (router must be running here)"
    ;;
  pi)
    _base_ip="${2:-}"
    if [ -z "${_base_ip}" ]; then
      echo "[zenoh-env] ERROR: pi role needs the base station IP:" >&2
      echo "             source deploy/scripts/zenoh-env.sh pi <basestation_ip>" >&2
    elif ! printf '%s' "${_base_ip}" | grep -Eq '^[0-9]{1,3}(\.[0-9]{1,3}){3}$'; then
      echo "[zenoh-env] ERROR: not a valid IPv4 address: ${_base_ip}" >&2
    else
      export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/${_base_ip}:7447\"]"
      echo "[zenoh-env] pi: RMW=rmw_zenoh_cpp DOMAIN=${ROS_DOMAIN_ID} -> router tcp/${_base_ip}:7447"
    fi
    ;;
  *)
    echo "[zenoh-env] usage: source ${BASH_SOURCE[0]:-zenoh-env.sh} <base|pi> [basestation_ip]" >&2
    ;;
esac
unset _zenoh_role _base_ip 2>/dev/null || true
