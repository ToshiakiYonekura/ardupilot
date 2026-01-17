#!/bin/bash
# Quick start script for multi-vehicle delivery mission
# This script assumes you're running Mission Planner separately

ARDUPILOT_ROOT="$(cd "$(dirname "$0")" && pwd)"

echo "Starting Multi-Vehicle Delivery Mission..."
echo "Connect Mission Planner to UDP ports: 14551 (Rover), 14561 (Boat), 14571 (Copter)"
echo ""

# Start each vehicle in background with screen (if available)
if command -v screen &> /dev/null; then
    echo "Using screen sessions..."

    # Rover
    screen -dmS delivery_rover bash -c "cd $ARDUPILOT_ROOT/Rover && ../Tools/autotest/sim_vehicle.py -v Rover -I 0 --sysid 1 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --custom-location=35.876991,140.348026,0,0 --console --map; exec bash"
    echo "✓ Rover started (screen -r delivery_rover)"

    # Boat
    screen -dmS delivery_boat bash -c "cd $ARDUPILOT_ROOT/Rover && ../Tools/autotest/sim_vehicle.py -v Rover -f motorboat -I 1 --sysid 2 --out 127.0.0.1:14560 --out 127.0.0.1:14561 --custom-location=35.879768,140.348495,0,0 --console --map; exec bash"
    echo "✓ Boat started (screen -r delivery_boat)"

    # Copter
    screen -dmS delivery_copter bash -c "cd $ARDUPILOT_ROOT/ArduCopter && ../Tools/autotest/sim_vehicle.py -v ArduCopter -I 2 --sysid 3 --out 127.0.0.1:14570 --out 127.0.0.1:14571 --custom-location=35.878275,140.338069,0,0 --console --map; exec bash"
    echo "✓ Copter started (screen -r delivery_copter)"

    echo ""
    echo "All vehicles starting... waiting 30 seconds for initialization..."
    sleep 30

    echo ""
    echo "Starting coordinator script..."
    python3 "$ARDUPILOT_ROOT/Tools/autotest/multi_vehicle_delivery.py"

    echo ""
    echo "Mission complete. To stop vehicles:"
    echo "  screen -S delivery_rover -X quit"
    echo "  screen -S delivery_boat -X quit"
    echo "  screen -S delivery_copter -X quit"

else
    echo "ERROR: screen not installed. Please install screen or use manual launch:"
    echo "  sudo apt-get install screen"
    echo ""
    echo "Or use the tmux option:"
    echo "  ./Tools/autotest/launch_delivery_simulation.sh --tmux"
fi
