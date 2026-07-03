#!/usr/bin/env bash
# Launch the Fixposition driver, auto-selecting how to reach the sensor:
#   1. WiFi  — via the sensor's mDNS name (fp-0e47c8.local); its IP is DHCP and
#              changes with the network, but the name is fixed.
#   2. Ethernet — fallback to the sensor's FIXED static IP (10.0.1.1); used when
#              WiFi isn't available/reachable.
# The first one that answers on the data port wins (WiFi preferred).
#
# The SDK's TCP client needs a numeric IP, so we resolve/choose one here, write a
# runtime config with it, and launch.
#
# Usage:  ./run_driver.sh
# Overrides: FP_SENSOR_HOST, FP_SENSOR_ETH_IP, FP_SENSOR_PORT
set -eo pipefail

SENSOR_HOST="${FP_SENSOR_HOST:-fp-0e47c8.local}"   # WiFi: mDNS name
ETH_IP="${FP_SENSOR_ETH_IP:-10.0.1.1}"             # Ethernet: fixed static IP
PORT="${FP_SENSOR_PORT:-21000}"
SRC_CONFIG="$HOME/fixposition_driver/fixposition_driver_ros2/launch/config.yaml"
RUN_DIR="$HOME/.config/fixposition"
RUN_CONFIG="$RUN_DIR/config.yaml"

# reachable <ip> : true if the sensor data port answers there
reachable() { timeout 2 bash -c "echo > /dev/tcp/$1/$PORT" 2>/dev/null; }

IP=""; VIA=""

# 1) WiFi first
echo "Trying WiFi (mDNS $SENSOR_HOST) ..."
# NOTE: '|| true' is required — under 'set -e' a failing resolve (e.g. wifi down)
# would otherwise abort the script instead of falling through to Ethernet.
WIFI_IP="$(timeout 4 getent hosts "$SENSOR_HOST" 2>/dev/null | awk '{print $1; exit}')" || true
if [ -n "$WIFI_IP" ] && reachable "$WIFI_IP"; then
    IP="$WIFI_IP"; VIA="WiFi ($SENSOR_HOST)"
else
    # 2) Ethernet fallback
    echo "  WiFi not reachable; trying Ethernet ($ETH_IP) ..."
    if reachable "$ETH_IP"; then
        IP="$ETH_IP"; VIA="Ethernet"
    fi
fi

if [ -z "$IP" ]; then
    echo "ERROR: sensor not reachable on WiFi ($SENSOR_HOST) or Ethernet ($ETH_IP:$PORT)."
    echo "  - Check the sensor is powered and connected."
    echo "  - WiFi test:     ping $SENSOR_HOST"
    echo "  - Ethernet test: ping $ETH_IP   (needs the cable + an IP on the 10.0.x net)"
    exit 1
fi
echo "Connecting via $VIA -> $IP:$PORT"

mkdir -p "$RUN_DIR"
# Base the runtime config on the repo config, overriding only the stream: line.
sed "s#^\( *stream:\).*#\1 tcpcli://$IP:$PORT#" "$SRC_CONFIG" > "$RUN_CONFIG"

# ROS setup scripts aren't -u safe; source them plainly (top has no -u).
source /opt/ros/humble/setup.bash
source "$HOME/fixposition_driver/install/setup.bash"
exec ros2 launch fixposition_driver_ros2 node.launch \
    config_dir:="$RUN_DIR" config:=config.yaml
