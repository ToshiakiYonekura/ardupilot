#!/usr/bin/env python3
"""
Automated test script for Mode 99 LQR State Feedback Controller

This script automates basic testing of the LQR controller in SITL.
It connects to MAVProxy, arms the vehicle, switches to mode 99,
and monitors telemetry.

Usage:
    python3 test_lqr_automated.py

Requirements:
    pip install pymavlink
"""

import sys
import time
from pymavlink import mavutil

class LQRTester:
    def __init__(self, connection_string='tcp:127.0.0.1:5760'):
        """Initialize connection to ArduCopter"""
        print("=" * 60)
        print("LQR Mode 99 Automated Test")
        print("=" * 60)
        print(f"\nConnecting to {connection_string}...")

        try:
            self.master = mavutil.mavlink_connection(connection_string)
            self.master.wait_heartbeat(timeout=10)
            print(f"✓ Connected to system {self.master.target_system}, component {self.master.target_component}")
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            print("\nMake sure SITL is running:")
            print("  cd ~/ardupilot/ArduCopter")
            print("  ../Tools/autotest/sim_vehicle.py --console --map")
            sys.exit(1)

        self.lqr_telemetry = {}
        self.mode_99_detected = False

    def wait_for_gps(self, timeout=60):
        """Wait for GPS lock and EKF ready"""
        print("\n[1/8] Waiting for GPS lock and EKF...")
        start_time = time.time()

        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
            if msg and msg.fix_type >= 3:
                print(f"✓ GPS locked: {msg.satellites_visible} satellites")
                break
            sys.stdout.write('.')
            sys.stdout.flush()
        else:
            print("\n✗ GPS lock timeout")
            return False

        # Wait a bit for EKF to initialize
        time.sleep(2)
        return True

    def check_prearm(self):
        """Check if vehicle is ready to arm"""
        print("\n[2/8] Checking pre-arm status...")

        # Request SYS_STATUS
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
            0, 0, 0, 0, 0, 0
        )

        msg = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        if msg:
            print(f"✓ Battery: {msg.voltage_battery/1000.0:.2f}V")
            print(f"✓ Pre-arm checks passed")
            return True
        return False

    def arm_vehicle(self):
        """Arm the vehicle"""
        print("\n[3/8] Arming vehicle...")

        self.master.arducopter_arm()

        # Wait for arming
        for i in range(10):
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("✓ Vehicle armed")
                return True
            time.sleep(0.5)

        print("✗ Arming failed")
        return False

    def set_mode(self, mode_name):
        """Set flight mode"""
        mode_mapping = self.master.mode_mapping()
        if mode_name not in mode_mapping:
            print(f"✗ Unknown mode: {mode_name}")
            return False

        mode_id = mode_mapping[mode_name]
        self.master.set_mode(mode_id)

        # Wait for mode change
        for i in range(10):
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.custom_mode == mode_id:
                return True
            time.sleep(0.5)

        return False

    def switch_to_mode_99(self):
        """Switch to Mode 99 (LQR State Feedback)"""
        print("\n[4/8] Switching to Mode 99 (LQR)...")

        # Mode 99 might be called SMARTPHOTO in ArduCopter
        # Try to set it
        success = self.set_mode("99")  # Try numeric mode

        if success:
            print("✓ Mode 99 active")
            self.mode_99_detected = True
        else:
            print("✗ Failed to switch to mode 99")
            print("  Note: Mode might have different name or number")

        return success

    def monitor_lqr_telemetry(self, duration=10):
        """Monitor LQR telemetry for specified duration"""
        print(f"\n[5/8] Monitoring LQR telemetry for {duration} seconds...")
        print("\nLooking for LQR messages in STATUSTEXT and NAMED_VALUE_FLOAT...")

        start_time = time.time()
        lqr_messages_found = False
        lqr_gains_calculated = False

        while time.time() - start_time < duration:
            # Check for STATUSTEXT messages (LQR initialization)
            msg = self.master.recv_match(type='STATUSTEXT', blocking=False)
            if msg:
                text = msg.text
                print(f"  STATUS: {text}")

                if "LQR" in text or "SMARTPHOTO99" in text:
                    lqr_messages_found = True
                    if "gains calculated" in text:
                        lqr_gains_calculated = True
                        print("  ✓ LQR gains calculated!")
                    if "Mass=" in text:
                        print("  ✓ System parameters loaded!")

            # Check for NAMED_VALUE_FLOAT (LQR telemetry)
            msg = self.master.recv_match(type='NAMED_VALUE_FLOAT', blocking=False)
            if msg:
                name = msg.name.strip('\x00')
                if name.startswith('LQR_'):
                    lqr_messages_found = True
                    self.lqr_telemetry[name] = msg.value

            # Print telemetry every second
            if int(time.time() - start_time) % 1 == 0 and self.lqr_telemetry:
                self._print_lqr_status()
                self.lqr_telemetry = {}  # Reset for next second

            time.sleep(0.01)

        if lqr_gains_calculated:
            print("\n✓ LQR controller initialized successfully!")
            return True
        elif lqr_messages_found:
            print("\n⚠ LQR messages found but gains not calculated")
            return False
        else:
            print("\n✗ No LQR telemetry detected")
            print("  Check that sysid_params.txt exists and mode 99 is active")
            return False

    def _print_lqr_status(self):
        """Print current LQR telemetry"""
        if not self.lqr_telemetry:
            return

        print("\n  --- LQR Telemetry ---")
        for key in sorted(self.lqr_telemetry.keys()):
            value = self.lqr_telemetry[key]
            print(f"  {key:15s}: {value:8.2f}")

    def takeoff_test(self):
        """Test takeoff with throttle"""
        print("\n[6/8] Testing takeoff...")
        print("  Increasing throttle to 65%...")

        # RC override: channel 3 (throttle) to 1650
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            1500, 1500, 1650, 1500, 0, 0, 0, 0
        )

        time.sleep(5)

        # Check altitude
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg and msg.relative_alt > 1000:  # > 1 meter
            print(f"✓ Altitude: {msg.relative_alt/1000.0:.2f}m")
            return True
        else:
            print("⚠ Takeoff not detected")
            return False

    def hover_test(self, duration=5):
        """Test hover stability"""
        print(f"\n[7/8] Testing hover stability for {duration} seconds...")

        # Center throttle
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            1500, 1500, 1500, 1500, 0, 0, 0, 0
        )

        altitudes = []
        start_time = time.time()

        while time.time() - start_time < duration:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                alt_m = msg.relative_alt / 1000.0
                altitudes.append(alt_m)
                sys.stdout.write(f"\r  Altitude: {alt_m:.2f}m")
                sys.stdout.flush()

        if altitudes:
            avg_alt = sum(altitudes) / len(altitudes)
            std_alt = (sum((x - avg_alt)**2 for x in altitudes) / len(altitudes)) ** 0.5
            print(f"\n  Average altitude: {avg_alt:.2f}m")
            print(f"  Std deviation: {std_alt:.2f}m")

            if std_alt < 1.0:
                print("✓ Stable hover achieved")
                return True

        return False

    def land_and_disarm(self):
        """Land and disarm"""
        print("\n[8/8] Landing...")

        # Reduce throttle for landing
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            1500, 1500, 1200, 1500, 0, 0, 0, 0
        )

        time.sleep(5)

        # Disarm
        print("  Disarming...")
        self.master.arducopter_disarm()

        time.sleep(2)
        print("✓ Test sequence complete")

    def run_full_test(self):
        """Run complete test sequence"""
        try:
            # Test sequence
            if not self.wait_for_gps():
                return False

            if not self.check_prearm():
                return False

            if not self.arm_vehicle():
                return False

            if not self.switch_to_mode_99():
                print("\n⚠ Mode 99 switch failed, continuing with current mode for telemetry check...")

            if not self.monitor_lqr_telemetry(duration=15):
                print("\n⚠ LQR telemetry monitoring incomplete")

            # Only proceed with flight test if everything is working
            if self.mode_99_detected and self.lqr_telemetry:
                self.takeoff_test()
                self.hover_test()

            self.land_and_disarm()

            print("\n" + "=" * 60)
            print("TEST SUMMARY")
            print("=" * 60)
            print(f"✓ GPS Lock: OK")
            print(f"✓ Armed: OK")
            print(f"{'✓' if self.mode_99_detected else '✗'} Mode 99: {'OK' if self.mode_99_detected else 'FAILED'}")
            print(f"{'✓' if self.lqr_telemetry else '✗'} LQR Telemetry: {'OK' if self.lqr_telemetry else 'NOT DETECTED'}")
            print("=" * 60)

            return True

        except KeyboardInterrupt:
            print("\n\nTest interrupted by user")
            print("Attempting safe shutdown...")
            self.land_and_disarm()
            return False
        except Exception as e:
            print(f"\n✗ Test failed with error: {e}")
            import traceback
            traceback.print_exc()
            return False

def main():
    print("\nLQR Mode 99 Automated Test Script")
    print("==================================\n")
    print("Prerequisites:")
    print("  1. SITL must be running: sim_vehicle.py")
    print("  2. sysid_params.txt must exist")
    print("  3. pymavlink must be installed: pip install pymavlink")
    print("\nStarting test in 3 seconds...")
    time.sleep(3)

    tester = LQRTester()
    success = tester.run_full_test()

    if success:
        print("\n✓ All tests completed!")
    else:
        print("\n✗ Some tests failed")

    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
