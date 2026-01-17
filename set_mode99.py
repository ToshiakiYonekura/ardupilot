#!/usr/bin/env python3
"""
Direct mode switch to Mode 99 via MAVLink
"""

from pymavlink import mavutil
import time

# Connect to SITL
master = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected to system {master.target_system}")

# Send DO_SET_MODE command to switch to mode 99
print("Sending command to switch to mode 99...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,  # confirmation
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # base mode
    99,  # custom mode (mode 99)
    0, 0, 0, 0, 0
)

# Wait for response
time.sleep(1)
msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
if msg:
    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("✓ Mode 99 activated!")
    else:
        print(f"✗ Mode switch failed: result={msg.result}")
else:
    print("✗ No response from vehicle")

# Check current mode
msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
if msg:
    print(f"Current mode number: {msg.custom_mode}")
    if msg.custom_mode == 99:
        print("✓ Successfully in mode 99!")
    else:
        print(f"✗ Still in mode {msg.custom_mode}")
