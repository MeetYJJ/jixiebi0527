#!/usr/bin/env bash
set -u

IF=${IF:-enP5p1s0f0}
ARM_IP=${ARM_IP:-192.168.1.18}
HOST_IP=${HOST_IP:-192.168.1.100}
HOST_CIDR=${HOST_CIDR:-192.168.1.100/24}
PORT=${PORT:-8080}

echo "======================================"
echo "T930 arm network check"
echo "IF        = $IF"
echo "HOST_IP   = $HOST_IP"
echo "ARM_IP    = $ARM_IP"
echo "PORT      = $PORT"
echo "======================================"

echo
echo "== 1. Check physical link =="
echo "carrier:"
cat "/sys/class/net/$IF/carrier" 2>/dev/null || echo "Cannot read carrier. Check interface name."
ip link show "$IF"

echo
echo "== 2. Configure T930 IP =="
sudo ip addr flush dev "$IF"
sudo ip link set "$IF" up
sudo ip addr add "$HOST_CIDR" dev "$IF"
ip -br addr show "$IF"

echo
echo "== 3. Configure route to arm =="
sudo ip route replace "$ARM_IP/32" dev "$IF" src "$HOST_IP"
ip route get "$ARM_IP"

echo
echo "== 4. Ping arm =="
ping -c 3 "$ARM_IP"

echo
echo "== 5. Check TCP port =="
nc -vz -w 3 "$ARM_IP" "$PORT"

echo
echo "== 6. Query arm state by JSON TCP =="
printf '{"command":"get_current_arm_state"}\r\n' | nc -w 3 "$ARM_IP" "$PORT"

echo
echo "== Done =="
