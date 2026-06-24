#!/usr/bin/env bash
# uninstall.sh <pi|laptop>
set -euo pipefail
ROLE="${1:?usage: uninstall.sh <pi|laptop>}"
if [[ "$ROLE" == "pi" ]]; then
  sudo systemctl disable --now scuttle-base.service || true
  sudo rm -f /etc/systemd/system/scuttle-base.service
  sudo systemctl daemon-reload
elif [[ "$ROLE" == "laptop" ]]; then
  systemctl --user disable --now scuttle-laptop-base.service zenoh-router.service || true
  rm -f "$HOME/.config/systemd/user/zenoh-router.service" \
        "$HOME/.config/systemd/user/scuttle-laptop-base.service"
  systemctl --user daemon-reload
else
  echo "unknown role: $ROLE" >&2; exit 1
fi
