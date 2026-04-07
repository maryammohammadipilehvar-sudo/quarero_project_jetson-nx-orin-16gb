#!/bin/bash
# Script to wait for network and LiDAR to be ready before starting ROS2

LIDAR_IP="192.168.10.3"
HOST_IP="192.168.10.226"
MAX_WAIT_SECONDS=120
WAIT_INTERVAL=2

echo "[wait_for_lidar] Waiting for network and LiDAR to be ready..."

# Function to check if host IP is bound to an interface
check_host_interface() {
    ip addr | grep -q "$HOST_IP"
}

# Function to check if LiDAR is pingable
check_lidar_ping() {
    ping -c 1 -W 1 "$LIDAR_IP" > /dev/null 2>&1
}

# Function to check if we can bind to a UDP port (test socket availability)
check_udp_bind() {
    # Try to create a test UDP socket - this verifies the network stack is ready
    timeout 1 bash -c "echo '' > /dev/udp/$HOST_IP/56999" 2>/dev/null
    return $?
}

waited=0

# Step 1: Wait for host interface to be ready
echo "[wait_for_lidar] Step 1: Waiting for host interface ($HOST_IP)..."
while ! check_host_interface; do
    if [ $waited -ge $MAX_WAIT_SECONDS ]; then
        echo "[wait_for_lidar] ERROR: Timeout waiting for host interface after ${MAX_WAIT_SECONDS}s"
        exit 1
    fi
    echo "[wait_for_lidar] Host interface not ready, waiting... (${waited}s/${MAX_WAIT_SECONDS}s)"
    sleep $WAIT_INTERVAL
    waited=$((waited + WAIT_INTERVAL))
done
echo "[wait_for_lidar] Host interface is ready!"

# Step 2: Wait for LiDAR to be pingable
echo "[wait_for_lidar] Step 2: Waiting for LiDAR ($LIDAR_IP) to respond to ping..."
while ! check_lidar_ping; do
    if [ $waited -ge $MAX_WAIT_SECONDS ]; then
        echo "[wait_for_lidar] ERROR: Timeout waiting for LiDAR after ${MAX_WAIT_SECONDS}s"
        exit 1
    fi
    echo "[wait_for_lidar] LiDAR not responding, waiting... (${waited}s/${MAX_WAIT_SECONDS}s)"
    sleep $WAIT_INTERVAL
    waited=$((waited + WAIT_INTERVAL))
done
echo "[wait_for_lidar] LiDAR is responding to ping!"

# Step 3: Additional stabilization delay (network stack needs time after interface is up)
STABILIZE_DELAY=5
echo "[wait_for_lidar] Step 3: Waiting ${STABILIZE_DELAY}s for network stack to stabilize..."
sleep $STABILIZE_DELAY

echo "[wait_for_lidar] All checks passed! Starting ROS2..."
exec "$@"
