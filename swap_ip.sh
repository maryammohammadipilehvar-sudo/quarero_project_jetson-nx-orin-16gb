#!/bin/bash
# Re-IP this Jetson (.29) → .226. Run ONLY after the old Jetson at .226 is unplugged/off.
# Per JETSON_TRANSFER.md §4 + HANDOFF.md.
#
# Usage:
#   ./swap_ip.sh
#
# What it does:
#   1. Refuses to run if .226 is still pingable (old Jetson must be off the LAN first).
#   2. nmcli mod: sets static 192.168.10.226/24, gw .1, dns 1.1.1.1/8.8.8.8.
#   3. nmcli up: applies. Your SSH session WILL DROP. Reconnect on .226.
#   4. Tells you what to restart after reconnect.
#
# Prereqs: sudo password ready. CLAUDE.md §3 — operator must approve each sudo.

set -e

CON="Wired connection 1"
NEW_IP="192.168.10.226"
GATEWAY="192.168.10.1"
DNS="1.1.1.1 8.8.8.8"

echo "[swap_ip] Step 1: confirm old Jetson at $NEW_IP is offline..."
if ping -c 2 -W 1 "$NEW_IP" > /dev/null 2>&1; then
  echo ""
  echo "!!! ABORT: $NEW_IP is still answering ping."
  echo "    Unplug the old Jetson's Ethernet (or poweroff) before running this."
  exit 1
fi
echo "[swap_ip] OK — $NEW_IP is free."
echo ""

echo "[swap_ip] Step 2: current ipv4 of '$CON':"
nmcli -t con show "$CON" | grep -E '^ipv4\.(method|addresses|gateway|dns):'
echo ""

echo "[swap_ip] Step 3: applying static config (will prompt for sudo)..."
sudo nmcli con mod "$CON" ipv4.addresses "$NEW_IP/24"
sudo nmcli con mod "$CON" ipv4.gateway "$GATEWAY"
sudo nmcli con mod "$CON" ipv4.dns "$DNS"
sudo nmcli con mod "$CON" ipv4.method manual
echo "[swap_ip] Config saved. Now bringing the connection up (SSH WILL DROP)..."
echo ""

# Detach the actual reconnect so SSH dropping doesn't leave nmcli half-run
nohup bash -c "sleep 1; sudo nmcli con up '$CON' >> /tmp/swap_ip.log 2>&1" > /dev/null 2>&1 &
echo "[swap_ip] nmcli con up scheduled. Reconnect at:"
echo "    ssh quarero02@$NEW_IP"
echo ""
echo "[swap_ip] After reconnect, run:"
echo "    bash ~/gits/restart_containers_after_ip_swap.sh"
