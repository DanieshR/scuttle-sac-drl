#!/usr/bin/env bash
# apply-sysctl.sh — install + load the ROS 2 UDP buffer tuning. Run on BOTH machines.
#   sudo ./deploy/scripts/apply-sysctl.sh
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # deploy/
src="${here}/config/sysctl/99-ros2-udp-buffers.conf"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo: sudo $0" >&2
  exit 1
fi

install -m 0644 "${src}" /etc/sysctl.d/99-ros2-udp-buffers.conf
sysctl --system >/dev/null
echo "Applied. Current values:"
sysctl net.core.rmem_max net.core.wmem_max
