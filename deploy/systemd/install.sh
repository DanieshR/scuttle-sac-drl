#!/usr/bin/env bash
# install.sh <pi|laptop> <laptop_ip> [ws_path]
#   pi     -> system service scuttle-base.service (needs sudo)
#   laptop -> user services zenoh-router + scuttle-laptop-base (+ linger)
set -euo pipefail

ROLE="${1:?usage: install.sh <pi|laptop> <laptop_ip> [ws_path]}"
LAPTOP_IP="${2:?need laptop_ip}"
WS="${3:-$HOME/ros2_ws}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

render() { sed -e "s|__USER__|$USER|g" -e "s|__WS__|$WS|g" -e "s|__LAPTOP_IP__|$LAPTOP_IP|g" "$1"; }

if [[ "$ROLE" == "pi" ]]; then
  render "$HERE/scuttle-base.service.in" | sudo tee /etc/systemd/system/scuttle-base.service >/dev/null
  sudo systemctl daemon-reload
  sudo systemctl enable --now scuttle-base.service
  echo "installed system service: scuttle-base.service"
elif [[ "$ROLE" == "laptop" ]]; then
  mkdir -p "$HOME/.config/systemd/user"
  render "$HERE/zenoh-router.service.in"        > "$HOME/.config/systemd/user/zenoh-router.service"
  render "$HERE/scuttle-laptop-base.service.in" > "$HOME/.config/systemd/user/scuttle-laptop-base.service"
  loginctl enable-linger "$USER"
  systemctl --user daemon-reload
  systemctl --user enable --now zenoh-router.service scuttle-laptop-base.service
  echo "installed user services: zenoh-router, scuttle-laptop-base"
else
  echo "unknown role: $ROLE" >&2; exit 1
fi
