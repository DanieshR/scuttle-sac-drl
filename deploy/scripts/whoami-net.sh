#!/usr/bin/env bash
# whoami-net.sh — print THIS machine's current hotspot/LAN IP.
#
# Why this exists: on a phone hotspot the DHCP lease changes every session, so
# NOTHING in this deployment may hardcode an IP. Every script and config that
# needs "the base station's address" reads it from here at runtime.
#
# Strategy: report the source IP the kernel would use to reach the outside world
# (i.e. the address bound to the default route). On a hotspot that is the
# hotspot-facing interface — exactly the address the *other* machine must target.
# This is more reliable than `hostname -I`, which also lists docker/bridge/loopback
# addresses and leaves you guessing which one is the real link.
set -euo pipefail

# Primary: source address of the default route (works whether or not the
# hotspot actually has upstream internet — we only need the route lookup).
ip=$(ip -4 route get 1.1.1.1 2>/dev/null | grep -oP 'src \K[0-9.]+' || true)

# Fallback: first non-loopback, non-docker IPv4 from hostname -I.
if [[ -z "${ip}" ]]; then
  for cand in $(hostname -I 2>/dev/null); do
    case "${cand}" in
      127.*|172.1[6-9].*|172.2[0-9].*|172.3[0-1].*) continue ;;  # loopback / docker
      *.*.*.*) ip="${cand}"; break ;;                            # first real IPv4
    esac
  done
fi

if [[ -z "${ip}" ]]; then
  echo "whoami-net: could not determine an IPv4 address" >&2
  exit 1
fi

echo "${ip}"
