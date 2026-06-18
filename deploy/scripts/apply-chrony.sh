#!/usr/bin/env bash
# apply-chrony.sh — install the right chrony config for this machine's role.
#
#   On the base station:  sudo ./deploy/scripts/apply-chrony.sh base
#   On the Pi:            sudo ./deploy/scripts/apply-chrony.sh pi <basestation_ip>
#
# Get <basestation_ip> by running ./deploy/scripts/whoami-net.sh on the base station.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # deploy/
role="${1:-}"
base_ip="${2:-}"
dest=/etc/chrony/conf.d/ros2-offload.conf

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo: sudo $0 ${role} ${base_ip}" >&2
  exit 1
fi
if [[ ! -d /etc/chrony/conf.d ]]; then
  echo "/etc/chrony/conf.d not found — is chrony installed? (sudo apt install chrony)" >&2
  exit 1
fi

case "${role}" in
  base)
    install -m 0644 "${here}/config/chrony/chrony-basestation.conf" "${dest}"
    ;;
  pi)
    if [[ -z "${base_ip}" ]]; then
      echo "Pi role needs the base station IP: sudo $0 pi <basestation_ip>" >&2
      exit 1
    fi
    # Validate dotted-quad before substituting into config.
    if [[ ! "${base_ip}" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]]; then
      echo "Not a valid IPv4 address: ${base_ip}" >&2
      exit 1
    fi
    sed "s/__BASESTATION_IP__/${base_ip}/" \
      "${here}/config/chrony/chrony-pi.conf.template" > "${dest}"
    chmod 0644 "${dest}"
    ;;
  *)
    echo "Usage: sudo $0 <base|pi> [basestation_ip]" >&2
    exit 1
    ;;
esac

systemctl restart chrony
echo "Installed ${dest} for role '${role}'. Waiting 3s for chrony..."
sleep 3
chronyc tracking || true
