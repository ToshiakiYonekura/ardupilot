#!/bin/bash
# Test script for Mode 99 LQR State Feedback Controller

echo "=========================================="
echo "Testing Mode 99 LQR State Feedback"
echo "=========================================="

# Check if sysid_params.txt exists
if [ ! -f "sysid_params.txt" ]; then
    echo "ERROR: sysid_params.txt not found!"
    echo "Please create system ID parameters file first."
    exit 1
fi

echo ""
echo "System ID parameters found:"
cat sysid_params.txt
echo ""

# Launch SITL
echo "Launching SITL..."
echo "Commands to test:"
echo "  1. arm throttle"
echo "  2. mode 99"
echo "  3. rc 3 1500  (center throttle)"
echo "  4. Observe LQR telemetry"
echo ""
echo "Expected telemetry:"
echo "  - SMARTPHOTO99: LQR gains calculated"
echo "  - Mass=2.00 kg, Hover=19.6 N"
echo "  - LQR_Thrust, LQR_M_roll, LQR_M_pitch, LQR_M_yaw"
echo "  - LQR_Rate: 100.0 Hz"
echo ""
echo "Press Ctrl+C to exit"
echo ""

cd ArduCopter
../Tools/autotest/sim_vehicle.py --console --map
