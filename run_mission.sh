#!/bin/bash
# Simple launcher for multi-vehicle mission with Mission Planner
# Runs all vehicles headless (no MAVProxy console/map)

ARDUPILOT_ROOT="$(cd "$(dirname "$0")" && pwd)"

echo "========================================="
echo "Multi-Vehicle Delivery Mission Launcher"
echo "========================================="
echo ""
echo "Starting vehicles in headless mode..."
echo "Connect Mission Planner to:"
echo "  - UDP 14551 (Rover)"
echo "  - UDP 14561 (Boat)"
echo "  - UDP 14571 (Copter)"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Stopping all vehicles..."
    pkill -f "ardurover.*-I0" 2>/dev/null
    pkill -f "ardurover.*-I1" 2>/dev/null
    pkill -f "arducopter.*-I2" 2>/dev/null
    pkill -f "mavproxy.py" 2>/dev/null
    exit 0
}

trap cleanup SIGINT SIGTERM

# Start Rover
echo "[1/3] Starting Rover..."
cd "$ARDUPILOT_ROOT/Rover"
../Tools/autotest/sim_vehicle.py -v Rover -I 0 --sysid 1 \
  --no-mavproxy \
  --out udp:127.0.0.1:14550 \
  --out udp:127.0.0.1:14551 \
  --custom-location=35.876991,140.348026,0,0 &
ROVER_PID=$!
sleep 2

# Start Boat
echo "[2/3] Starting Boat..."
cd "$ARDUPILOT_ROOT/Rover"
../Tools/autotest/sim_vehicle.py -v Rover -f motorboat -I 1 --sysid 2 \
  --no-mavproxy \
  --out udp:127.0.0.1:14560 \
  --out udp:127.0.0.1:14561 \
  --custom-location=35.879768,140.348495,0,0 &
BOAT_PID=$!
sleep 2

# Start Copter
echo "[3/3] Starting Copter..."
cd "$ARDUPILOT_ROOT/ArduCopter"
../Tools/autotest/sim_vehicle.py -v ArduCopter -I 2 --sysid 3 \
  --no-mavproxy \
  --out udp:127.0.0.1:14570 \
  --out udp:127.0.0.1:14571 \
  --custom-location=35.878275,140.338069,0,0 &
COPTER_PID=$!

echo ""
echo "✓ All vehicles starting..."
echo "  Rover PID: $ROVER_PID"
echo "  Boat PID: $BOAT_PID"
echo "  Copter PID: $COPTER_PID"
echo ""
echo "Waiting 45 seconds for initialization..."

sleep 45

echo ""
echo "========================================="
echo "Starting Coordinator Script"
echo "========================================="
echo ""

cd "$ARDUPILOT_ROOT"
python3 Tools/autotest/multi_vehicle_delivery.py

echo ""
echo "Mission complete!"
cleanup
