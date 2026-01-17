#!/bin/bash

# Multi-Vehicle Delivery System - SITL Launch Script
#
# This script provides commands to launch the multi-vehicle delivery simulation.
# You need to run each command in a SEPARATE TERMINAL window.
#
# Usage:
#   1. Open 4 terminal windows
#   2. Copy and run each command below in separate terminals
#   3. Wait for all vehicles to be ready (check for "armable" message)
#   4. Run the coordinator script in terminal 4

# Get the ArduPilot root directory (assumes this script is in Tools/autotest/)
ARDUPILOT_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

echo "========================================="
echo "Multi-Vehicle Delivery System - SITL Launcher"
echo "========================================="
echo ""
echo "ArduPilot root: $ARDUPILOT_ROOT"
echo ""
echo "You need to open 4 SEPARATE terminal windows and run these commands:"
echo ""
echo "========================================="
echo "TERMINAL 1: Launch Rover"
echo "========================================="
echo "cd $ARDUPILOT_ROOT/Rover"
echo "../Tools/autotest/sim_vehicle.py -v Rover -I 0 --sysid 1 \\"
echo "  --out 127.0.0.1:14550 \\"
echo "  --custom-location=35.876991,140.348026,0,0 \\"
echo "  --console --map"
echo ""
echo "========================================="
echo "TERMINAL 2: Launch Boat (Motorboat)"
echo "========================================="
echo "cd $ARDUPILOT_ROOT/Rover"
echo "../Tools/autotest/sim_vehicle.py -v Rover -f motorboat -I 1 --sysid 2 \\"
echo "  --out 127.0.0.1:14560 \\"
echo "  --custom-location=35.879768,140.348495,0,0 \\"
echo "  --console --map"
echo ""
echo "========================================="
echo "TERMINAL 3: Launch Copter"
echo "========================================="
echo "cd $ARDUPILOT_ROOT/ArduCopter"
echo "../Tools/autotest/sim_vehicle.py -v ArduCopter -I 2 --sysid 3 \\"
echo "  --out 127.0.0.1:14570 \\"
echo "  --custom-location=35.878275,140.338069,0,0 \\"
echo "  --console --map"
echo ""
echo "========================================="
echo "TERMINAL 4: Run Coordinator (after all vehicles are ready)"
echo "========================================="
echo "cd $ARDUPILOT_ROOT"
echo "python3 Tools/autotest/multi_vehicle_delivery.py"
echo ""
echo "========================================="
echo "Notes:"
echo "========================================="
echo "- Wait for all vehicles to show 'armable' before running coordinator"
echo "- Each vehicle will open a MAVProxy console and map window"
echo "- The coordinator script will automatically coordinate the mission"
echo "- Press Ctrl+C in the coordinator terminal to stop the mission"
echo "- Close each SITL instance when done (Ctrl+C in each terminal)"
echo ""
echo "Mission Route:"
echo "  1. Rover: Namekawa Station → Opposite Shore Port"
echo "  2. Boat: Opposite Shore Port → Main Port (waits for Rover)"
echo "  3. Copter: Main Port → Seven-Eleven (waits for Boat)"
echo ""
echo "========================================="
echo "Port Assignments:"
echo "========================================="
echo "Rover:  TCP 5760, UDP 14550 (DroneKit)"
echo "Boat:   TCP 5770, UDP 14560 (DroneKit)"
echo "Copter: TCP 5780, UDP 14570 (DroneKit)"
echo "========================================="
echo ""
echo "For automated launch using tmux (advanced):"
echo "Run: $0 --tmux"
echo ""

# Function to launch all in tmux
launch_tmux() {
    echo "Launching in tmux session 'delivery'..."

    # Check if tmux is installed
    if ! command -v tmux &> /dev/null; then
        echo "ERROR: tmux is not installed. Please install it first:"
        echo "  sudo apt-get install tmux"
        exit 1
    fi

    # Kill existing session if it exists
    tmux kill-session -t delivery 2>/dev/null

    # Create new session and windows
    tmux new-session -d -s delivery -n rover

    # Window 0: Rover
    tmux send-keys -t delivery:0 "cd $ARDUPILOT_ROOT/Rover" C-m
    tmux send-keys -t delivery:0 "../Tools/autotest/sim_vehicle.py -v Rover -I 0 --sysid 1 --out 127.0.0.1:14550 --custom-location=35.876991,140.348026,0,0 --console --map" C-m

    # Window 1: Boat
    tmux new-window -t delivery -n boat
    tmux send-keys -t delivery:1 "cd $ARDUPILOT_ROOT/Rover" C-m
    tmux send-keys -t delivery:1 "../Tools/autotest/sim_vehicle.py -v Rover -f motorboat -I 1 --sysid 2 --out 127.0.0.1:14560 --custom-location=35.879768,140.348495,0,0 --console --map" C-m

    # Window 2: Copter
    tmux new-window -t delivery -n copter
    tmux send-keys -t delivery:2 "cd $ARDUPILOT_ROOT/ArduCopter" C-m
    tmux send-keys -t delivery:2 "../Tools/autotest/sim_vehicle.py -v ArduCopter -I 2 --sysid 3 --out 127.0.0.1:14570 --custom-location=35.878275,140.338069,0,0 --console --map" C-m

    # Window 3: Coordinator (but don't run yet)
    tmux new-window -t delivery -n coordinator
    tmux send-keys -t delivery:3 "cd $ARDUPILOT_ROOT" C-m
    tmux send-keys -t delivery:3 "echo 'Waiting for vehicles to be ready...'" C-m
    tmux send-keys -t delivery:3 "echo 'When all vehicles show armable, run:'" C-m
    tmux send-keys -t delivery:3 "echo '  python3 Tools/autotest/multi_vehicle_delivery.py'" C-m

    echo ""
    echo "Tmux session 'delivery' created with 4 windows:"
    echo "  Window 0: rover"
    echo "  Window 1: boat"
    echo "  Window 2: copter"
    echo "  Window 3: coordinator"
    echo ""
    echo "Attach to the session with:"
    echo "  tmux attach -t delivery"
    echo ""
    echo "Navigate between windows:"
    echo "  Ctrl+b then 0,1,2,3  - Switch to window"
    echo "  Ctrl+b then n        - Next window"
    echo "  Ctrl+b then p        - Previous window"
    echo ""
    echo "When ready, switch to window 3 and run the coordinator:"
    echo "  python3 Tools/autotest/multi_vehicle_delivery.py"
    echo ""
    echo "To detach from session: Ctrl+b then d"
    echo "To kill session: tmux kill-session -t delivery"
    echo ""
}

# Check for tmux flag
if [ "$1" == "--tmux" ]; then
    launch_tmux
    exit 0
fi

# If no arguments, just show instructions (already printed above)
exit 0
