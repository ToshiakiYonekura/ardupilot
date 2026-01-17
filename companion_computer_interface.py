#!/usr/bin/env python3
"""
Companion Computer Interface for Mode 99 LQR Controller

This script provides a complete interface for controlling ArduCopter Mode 99
from a companion computer (Raspberry Pi, Jetson, etc.)

Features:
  - Send position/velocity commands to LQR controller
  - Monitor mission state machine
  - Receive wind estimates
  - Plan and execute autonomous missions
  - Safety monitoring and emergency handling

Requirements:
    pip install pymavlink numpy

Usage:
    python3 companion_computer_interface.py --connect /dev/ttyAMA0

Author: ArduPilot LQR Controller Team
Date: January 2026
"""

import sys
import time
import argparse
import math
import numpy as np
from pymavlink import mavutil

class Mode99CompanionInterface:
    """Interface for controlling ArduCopter Mode 99 from companion computer"""

    def __init__(self, connection_string):
        """Initialize connection to flight controller"""
        print("=" * 70)
        print("Mode 99 LQR Controller - Companion Computer Interface")
        print("=" * 70)
        print(f"\nConnecting to {connection_string}...")

        self.master = mavutil.mavlink_connection(connection_string, baud=57600)
        self.master.wait_heartbeat(timeout=10)

        print(f"✓ Connected to system {self.master.target_system}, "
              f"component {self.master.target_component}")

        # State tracking
        self.current_mode = None
        self.mission_state = 0  # Mode 99 state
        self.position_ned = np.zeros(3)
        self.velocity_ned = np.zeros(3)
        self.attitude = np.zeros(3)  # [roll, pitch, yaw]
        self.wind_estimate = np.zeros(3)
        self.gps_fix = 0
        self.armed = False

        # Mission waypoints
        self.waypoints = []
        self.current_waypoint_idx = 0

        # Timing
        self.last_command_time = 0
        self.command_rate_hz = 10  # Send commands at 10Hz

        # Safety limits
        self.max_velocity = 5.0  # m/s
        self.max_altitude = 100.0  # m
        self.min_altitude = 0.5  # m

    def request_data_streams(self):
        """Request necessary data streams from flight controller"""
        print("\nRequesting data streams...")

        # Request position data at 10Hz
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            10,  # Hz
            1    # Enable
        )

        # Request extra data (wind, named values) at 10Hz
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            10,
            1
        )

        print("✓ Data streams requested")

    def wait_for_mode_99(self, timeout=30):
        """Wait for vehicle to enter Mode 99"""
        print(f"\nWaiting for Mode 99 (timeout: {timeout}s)...")
        start_time = time.time()

        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                # Mode 99 = custom mode number 99
                if msg.custom_mode == 99:
                    print("✓ Mode 99 active!")
                    self.current_mode = 99
                    return True
                else:
                    sys.stdout.write(f"\rCurrent mode: {msg.custom_mode}")
                    sys.stdout.flush()

        print("\n✗ Timeout waiting for Mode 99")
        return False

    def send_route_set_command(self):
        """Send ROUTE_SET command to indicate mission is ready"""
        print("\nSending ROUTE_SET command...")

        # Use MAV_CMD_USER_1 as ROUTE_SET signal
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_USER_1,  # ROUTE_SET command
            0,  # confirmation
            1,  # param1: ROUTE_SET signal
            0, 0, 0, 0, 0, 0
        )

        # Wait for acknowledgment
        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("✓ ROUTE_SET acknowledged")
            return True
        else:
            print("✗ ROUTE_SET not acknowledged")
            return False

    def send_position_velocity_command(self, pos_ned, vel_ned, yaw_rad, yaw_rate_rad_s):
        """
        Send position and velocity command to Mode 99 LQR controller

        Args:
            pos_ned: [north, east, down] position in meters (NED frame)
            vel_ned: [north, east, down] velocity in m/s (NED frame)
            yaw_rad: Target yaw in radians (0 = North)
            yaw_rate_rad_s: Target yaw rate in rad/s

        Units (CRITICAL):
            Position: meters (NED)
            Velocity: m/s (NED)
            Yaw: radians
            Yaw rate: rad/s
        """

        # Safety checks
        if abs(pos_ned[2]) > self.max_altitude:
            print(f"⚠ Altitude limit exceeded: {abs(pos_ned[2])}m > {self.max_altitude}m")
            pos_ned[2] = -self.max_altitude if pos_ned[2] < 0 else -self.min_altitude

        vel_norm = np.linalg.norm(vel_ned)
        if vel_norm > self.max_velocity:
            print(f"⚠ Velocity limit exceeded: {vel_norm:.2f}m/s > {self.max_velocity}m/s")
            vel_ned = vel_ned * (self.max_velocity / vel_norm)

        # Send SET_POSITION_TARGET_LOCAL_NED message
        # Type mask: 0b0000111111000111 = 0x0FC7 (ignore accel, force, yaw)
        # We're using position, velocity, and yaw rate
        type_mask = (
            # Position: use x, y, z
            0 |  # bit 0: x
            0 |  # bit 1: y
            0 |  # bit 2: z
            # Velocity: use vx, vy, vz
            0 |  # bit 3: vx
            0 |  # bit 4: vy
            0 |  # bit 5: vz
            # Acceleration: ignore
            (1 << 6) |  # bit 6: afx
            (1 << 7) |  # bit 7: afy
            (1 << 8) |  # bit 8: afz
            # Force: ignore
            (1 << 9) |  # bit 9: force
            # Yaw: use yaw rate, ignore yaw angle for now
            (1 << 10) |  # bit 10: yaw (ignore)
            0            # bit 11: yaw_rate (use)
        )

        self.master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000),  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            pos_ned[0], pos_ned[1], pos_ned[2],  # x, y, z [m]
            vel_ned[0], vel_ned[1], vel_ned[2],  # vx, vy, vz [m/s]
            0, 0, 0,  # afx, afy, afz (ignored)
            yaw_rad,  # yaw [rad]
            yaw_rate_rad_s  # yaw_rate [rad/s]
        )

        self.last_command_time = time.time()

    def update_telemetry(self):
        """Update state from incoming messages"""

        # Non-blocking receive
        msg = self.master.recv_match(blocking=False)
        if not msg:
            return

        msg_type = msg.get_type()

        # Position
        if msg_type == 'LOCAL_POSITION_NED':
            self.position_ned = np.array([msg.x, msg.y, msg.z])
            self.velocity_ned = np.array([msg.vx, msg.vy, msg.vz])

        # Attitude
        elif msg_type == 'ATTITUDE':
            self.attitude = np.array([msg.roll, msg.pitch, msg.yaw])

        # GPS status
        elif msg_type == 'GPS_RAW_INT':
            self.gps_fix = msg.fix_type

        # Named float values (LQR telemetry, wind)
        elif msg_type == 'NAMED_VALUE_FLOAT':
            name = msg.name.strip('\x00')

            if name == 'Mode99State':
                self.mission_state = int(msg.value)

            elif name.startswith('Wind'):
                if name == 'WindN':
                    self.wind_estimate[0] = msg.value
                elif name == 'WindE':
                    self.wind_estimate[1] = msg.value
                elif name == 'WindD':
                    self.wind_estimate[2] = msg.value

        # Heartbeat
        elif msg_type == 'HEARTBEAT':
            self.current_mode = msg.custom_mode
            self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

    def print_status(self):
        """Print current status"""
        print(f"\r"
              f"Mode: {self.current_mode:3d} | "
              f"State: {self.mission_state} | "
              f"Pos: [{self.position_ned[0]:6.2f}, {self.position_ned[1]:6.2f}, {self.position_ned[2]:6.2f}] | "
              f"Vel: [{self.velocity_ned[0]:5.2f}, {self.velocity_ned[1]:5.2f}, {self.velocity_ned[2]:5.2f}] | "
              f"Wind: [{self.wind_estimate[0]:4.1f}, {self.wind_estimate[1]:4.1f}]",
              end='')
        sys.stdout.flush()

    def load_mission_waypoints(self, waypoints):
        """
        Load mission waypoints

        Args:
            waypoints: List of (north, east, down, yaw) tuples in meters and radians
        """
        self.waypoints = waypoints
        self.current_waypoint_idx = 0
        print(f"\n✓ Loaded {len(waypoints)} waypoints")

    def execute_mission(self):
        """Execute waypoint mission"""
        print("\n" + "=" * 70)
        print("EXECUTING MISSION")
        print("=" * 70)

        if not self.waypoints:
            print("✗ No waypoints loaded")
            return False

        # Send ROUTE_SET
        if not self.send_route_set_command():
            print("✗ Failed to send ROUTE_SET")
            return False

        time.sleep(1)

        # Execute waypoints
        for idx, waypoint in enumerate(self.waypoints):
            print(f"\n--- Waypoint {idx+1}/{len(self.waypoints)} ---")
            self.current_waypoint_idx = idx

            target_pos = np.array(waypoint[:3])
            target_yaw = waypoint[3]

            print(f"Target: N={target_pos[0]:.1f}m, E={target_pos[1]:.1f}m, "
                  f"D={target_pos[2]:.1f}m, Yaw={math.degrees(target_yaw):.0f}°")

            # Navigate to waypoint
            if not self.navigate_to_position(target_pos, target_yaw):
                print("✗ Navigation failed")
                return False

        print("\n" + "=" * 70)
        print("✓ MISSION COMPLETE")
        print("=" * 70)
        return True

    def navigate_to_position(self, target_pos, target_yaw, timeout=60):
        """Navigate to a target position"""

        threshold = 1.0  # m
        start_time = time.time()

        while time.time() - start_time < timeout:
            # Update telemetry
            self.update_telemetry()

            # Check distance to target
            error = target_pos - self.position_ned
            distance = np.linalg.norm(error)

            if distance < threshold:
                print(f"\n✓ Reached waypoint (error: {distance:.2f}m)")
                return True

            # Calculate velocity command (simple proportional)
            vel_gain = 0.5
            vel_cmd = error * vel_gain
            vel_cmd = np.clip(vel_cmd, -self.max_velocity, self.max_velocity)

            # Send command at 10Hz
            current_time = time.time()
            if current_time - self.last_command_time >= 1.0 / self.command_rate_hz:
                self.send_position_velocity_command(
                    target_pos,
                    vel_cmd,
                    target_yaw,
                    0.0  # No yaw rate
                )

            # Print status
            self.print_status()

            time.sleep(0.05)  # 20Hz update

        print(f"\n✗ Timeout (distance: {distance:.2f}m)")
        return False

    def emergency_land(self):
        """Command emergency landing"""
        print("\n⚠ EMERGENCY LANDING ⚠")

        # Send descending commands
        for i in range(50):  # 5 seconds at 10Hz
            current_pos = self.position_ned.copy()
            current_pos[2] += 1.0  # Descend 1m/s

            self.send_position_velocity_command(
                current_pos,
                np.array([0, 0, 1.0]),  # 1m/s down
                self.attitude[2],  # Current yaw
                0.0
            )

            time.sleep(0.1)

    def run_test_mission(self):
        """Run a simple test mission"""
        print("\n" + "=" * 70)
        print("TEST MISSION: Square Pattern")
        print("=" * 70)

        # Define square waypoints (10m sides at 20m altitude)
        altitude = -20.0  # Negative in NED (up is negative)
        side_length = 10.0

        waypoints = [
            (0.0, 0.0, altitude, 0.0),                              # Start
            (side_length, 0.0, altitude, 0.0),                      # North
            (side_length, side_length, altitude, math.pi/2),        # East
            (0.0, side_length, altitude, math.pi),                  # South
            (0.0, 0.0, altitude, -math.pi/2),                       # West (back to start)
        ]

        self.load_mission_waypoints(waypoints)
        return self.execute_mission()


def main():
    parser = argparse.ArgumentParser(description='Mode 99 Companion Computer Interface')
    parser.add_argument('--connect', default='tcp:127.0.0.1:5760',
                       help='Connection string (default: tcp:127.0.0.1:5760)')
    parser.add_argument('--mission', choices=['test', 'custom'], default='test',
                       help='Mission type to run')
    args = parser.parse_args()

    try:
        # Initialize interface
        interface = Mode99CompanionInterface(args.connect)

        # Request data streams
        interface.request_data_streams()

        # Wait for Mode 99
        if not interface.wait_for_mode_99():
            print("\n✗ Please switch to Mode 99 manually")
            return 1

        # Run mission
        if args.mission == 'test':
            success = interface.run_test_mission()
        else:
            print("\nCustom mission not implemented")
            success = False

        if success:
            print("\n✓ Mission completed successfully")
            return 0
        else:
            print("\n✗ Mission failed")
            return 1

    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")
        interface.emergency_land()
        return 1

    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
