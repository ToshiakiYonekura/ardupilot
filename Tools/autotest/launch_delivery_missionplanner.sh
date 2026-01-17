#!/bin/bash

# Multi-Vehicle Delivery System - Mission Planner Compatible Launch
#
# This script launches SITL instances configured to work with Mission Planner
# for visualization while the coordinator script manages the mission.

# Get the ArduPilot root directory
ARDUPILOT_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

echo "========================================="
echo "Multi-Vehicle Delivery - Mission Planner Setup"
echo "========================================="
echo ""
echo "This setup allows Mission Planner to visualize vehicles while"
echo "the coordinator script manages the autonomous mission."
echo ""
echo "Mission Planner Connection (TCP Ports):"
echo "  - Rover: 5760"
echo "  - Boat: 5770"
echo "  - Copter: 5780"
echo ""

# Function to launch all vehicles
launch_vehicles() {
    echo "========================================="
    echo "TERMINAL 1: Launch Rover"
    echo "========================================="
    echo "cd $ARDUPILOT_ROOT/Rover"
    echo "../Tools/autotest/sim_vehicle.py -v Rover -I 0 --sysid 1 \\"
    echo "  --custom-location=35.876991,140.348026,0,0 \\"
    echo "  --console --map"
    echo ""
    echo "Mission Planner: Connect to TCP port 5760"
    echo ""
    echo "========================================="
    echo "TERMINAL 2: Launch Boat"
    echo "========================================="
    echo "cd $ARDUPILOT_ROOT/Rover"
    echo "../Tools/autotest/sim_vehicle.py -v Rover -f motorboat -I 1 --sysid 2 \\"
    echo "  --custom-location=35.879768,140.348495,0,0 \\"
    echo "  --console --map"
    echo ""
    echo "Mission Planner: Connect to TCP port 5770"
    echo ""
    echo "========================================="
    echo "TERMINAL 3: Launch Copter"
    echo "========================================="
    echo "cd $ARDUPILOT_ROOT/ArduCopter"
    echo "../Tools/autotest/sim_vehicle.py -v ArduCopter -I 2 --sysid 3 \\"
    echo "  --custom-location=35.878275,140.338069,0,0 \\"
    echo "  --console --map"
    echo ""
    echo "Mission Planner: Connect to TCP port 5780"
    echo ""
    echo "========================================="
    echo "TERMINAL 4: Run Coordinator"
    echo "========================================="
    echo "cd $ARDUPILOT_ROOT"
    echo "python3 Tools/autotest/multi_vehicle_delivery.py"
    echo ""
    echo "========================================="
}

# Function for Mission Planner connection instructions
missionplanner_instructions() {
    echo "========================================="
    echo "Mission Planner Connection Guide"
    echo "========================================="
    echo ""
    echo "OPTION 1: Connect to Single Vehicle"
    echo "------------------------------------"
    echo "1. Launch all 3 SITL instances (see commands above)"
    echo "2. Open Mission Planner"
    echo "3. Top-right corner: Select 'TCP' from dropdown"
    echo "4. Click 'Connect'"
    echo "5. In the popup, enter port number:"
    echo "   - Rover: 5760"
    echo "   - Boat: 5770"
    echo "   - Copter: 5780"
    echo "6. Click 'OK' to connect"
    echo ""
    echo "To switch vehicles:"
    echo "  - Disconnect from current vehicle"
    echo "  - Click 'Connect' and enter different port"
    echo ""
    echo "OPTION 2: Run Multiple Mission Planner Instances"
    echo "------------------------------------------------"
    echo "1. Open 3 separate Mission Planner windows"
    echo "2. Connect each to different TCP port (5760, 5770, 5780)"
    echo "3. Arrange windows to view all vehicles simultaneously"
    echo ""
    echo "OPTION 3: Use QGroundControl (Recommended)"
    echo "-------------------------------------------"
    echo "QGroundControl has native multi-vehicle support:"
    echo "1. Launch all SITL instances"
    echo "2. Open QGroundControl"
    echo "3. Go to: Application Settings → Comm Links"
    echo "4. Add TCP connections:"
    echo "   - Link 1: TCP Port 5760 (Rover)"
    echo "   - Link 2: TCP Port 5770 (Boat)"
    echo "   - Link 3: TCP Port 5780 (Copter)"
    echo "5. All vehicles appear in vehicle selector dropdown"
    echo ""
    echo "Download QGroundControl:"
    echo "  https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html"
    echo ""
}

# Function to create a start script
create_start_script() {
    cat > "$ARDUPILOT_ROOT/start_delivery_mission.sh" << 'EOF'
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
EOF
    chmod +x "$ARDUPILOT_ROOT/start_delivery_mission.sh"
    echo "✓ Created $ARDUPILOT_ROOT/start_delivery_mission.sh"
}

# Main menu
echo "What would you like to do?"
echo ""
echo "1) Show launch commands (manual terminal setup)"
echo "2) Show Mission Planner connection instructions"
echo "3) Create automated start script (requires screen)"
echo "4) All of the above"
echo ""
read -p "Enter choice [1-4]: " choice

case $choice in
    1)
        launch_vehicles
        ;;
    2)
        missionplanner_instructions
        ;;
    3)
        create_start_script
        echo ""
        echo "Run the mission with:"
        echo "  ./start_delivery_mission.sh"
        ;;
    4)
        launch_vehicles
        echo ""
        missionplanner_instructions
        echo ""
        create_start_script
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac

echo ""
echo "========================================="
echo "Port Reference"
echo "========================================="
echo "Rover:"
echo "  - TCP Port: 5760 (Mission Planner & DroneKit)"
echo "  - SYSID: 1"
echo ""
echo "Boat:"
echo "  - TCP Port: 5770 (Mission Planner & DroneKit)"
echo "  - SYSID: 2"
echo ""
echo "Copter:"
echo "  - TCP Port: 5780 (Mission Planner & DroneKit)"
echo "  - SYSID: 3"
echo "========================================="
