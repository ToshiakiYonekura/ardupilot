#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-Vehicle Autonomous Delivery System

This script coordinates three vehicle types (Rover, Boat, Copter) to perform
an automated delivery sequence with cargo handoffs between vehicles.

Route:
1. Rover: Namekawa Station → Opposite Shore Port
2. Boat: Opposite Shore Port → Main Port (waits for Rover)
3. Copter: Main Port → Seven-Eleven (waits for Boat)

Author: ArduPilot Multi-Vehicle Delivery System
"""

import math
import time
from dronekit import connect, LocationGlobalRelative, VehicleMode

# Configuration
WAIT_READY = False  # Set to True to wait for all parameters to download
TOWER_LOOP_INTERVAL = 5  # seconds
ARRIVAL_THRESHOLD = 10  # meters
COPTER_TAKEOFF_ALT = 20  # meters

# Vehicle connection ports (TCP)
ROVER_PORT = 'tcp:127.0.0.1:5760'
BOAT_PORT = 'tcp:127.0.0.1:5770'
COPTER_PORT = 'tcp:127.0.0.1:5780'


def log(msg):
    """Print timestamped log message"""
    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
    print(f'[{timestamp}] {msg}')


def prepare_vehicles():
    """Connect to all three vehicles and wait until ready"""
    log('Connecting to vehicles...')

    rover = connect(ROVER_PORT, wait_ready=WAIT_READY, timeout=60)
    log('Rover connected')

    boat = connect(BOAT_PORT, wait_ready=WAIT_READY, timeout=60)
    log('Boat connected')

    copter = connect(COPTER_PORT, wait_ready=WAIT_READY, timeout=60)
    log('Copter connected')

    log('All vehicles connected. Waiting for armability...')

    rover.wait_for_armable()
    log('Rover is armable')

    boat.wait_for_armable()
    log('Boat is armable')

    copter.wait_for_armable()
    log('Copter is armable')

    # Add vehicle identifiers for logging
    rover.vehicle_name = 'Rover'
    boat.vehicle_name = 'Boat'
    copter.vehicle_name = 'Copter'

    return rover, boat, copter


class ControlTower:
    """Central coordinator for multi-vehicle delivery mission"""

    # Mission routes with handoff dependencies
    routes = {
        'rover': {
            'vehicle': None,
            'from': [35.876991, 140.348026, 0],  # Namekawa Station (pickup)
            'to': [35.879768, 140.348495, 0],     # Opposite Shore Port
            'wait_for': None  # Starts immediately
        },
        'boat': {
            'vehicle': None,
            'from': [35.879768, 140.348495, 0],   # Opposite Shore Port
            'to': [35.878275, 140.338069, 0],     # Main Port
            'wait_for': 'rover'  # Waits for rover to arrive
        },
        'copter': {
            'vehicle': None,
            'from': [35.878275, 140.338069, 0],   # Main Port
            'to': [35.877518, 140.295439, 50],    # Seven-Eleven (delivery)
            'wait_for': 'boat'  # Waits for boat to arrive
        }
    }

    def __init__(self, rover, boat, copter):
        """Initialize control tower with vehicle instances"""
        self.routes['rover']['vehicle'] = rover
        self.routes['boat']['vehicle'] = boat
        self.routes['copter']['vehicle'] = copter

        # Track mission status
        self.is_complete = False
        self.launched_vehicles = set()

    def _calculate_distance(self, vehicle, target_location):
        """
        Calculate distance in meters between vehicle and target using
        haversine approximation (suitable for short distances)

        Args:
            vehicle: DroneKit vehicle instance
            target_location: LocationGlobalRelative object

        Returns:
            Distance in meters
        """
        dlat = vehicle.location.global_frame.lat - target_location.lat
        dlong = vehicle.location.global_frame.lon - target_location.lon

        # Convert lat/lon degrees to meters (approximate)
        # 1 degree ≈ 111.32 km at equator
        distance = math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

        return distance

    def _is_arrived(self, vehicle, destination):
        """
        Check if vehicle has arrived at destination

        Args:
            vehicle: DroneKit vehicle instance
            destination: [lat, lon, alt] list

        Returns:
            True if within ARRIVAL_THRESHOLD meters
        """
        target = LocationGlobalRelative(destination[0], destination[1], destination[2])
        distance = self._calculate_distance(vehicle, target)

        return distance <= ARRIVAL_THRESHOLD

    def _launch_vehicle(self, vehicle_name):
        """
        Launch a vehicle by arming and setting mode to GUIDED

        Args:
            vehicle_name: Name of vehicle to launch ('rover', 'boat', 'copter')
        """
        vehicle = self.routes[vehicle_name]['vehicle']

        log(f'Launching {vehicle_name}...')

        # Set mode to GUIDED
        vehicle.mode = VehicleMode('GUIDED')
        time.sleep(2)  # Allow mode change to settle

        # Arm the vehicle
        if not vehicle.armed:
            log(f'Arming {vehicle_name}...')
            vehicle.armed = True

            # Wait for arming (with timeout)
            timeout = 30
            start_time = time.time()
            while not vehicle.armed and (time.time() - start_time) < timeout:
                log(f'Waiting for {vehicle_name} to arm...')
                time.sleep(1)

            if not vehicle.armed:
                log(f'ERROR: Failed to arm {vehicle_name}')
                return False

            log(f'{vehicle_name} armed successfully')

        # For copter, takeoff first
        if vehicle_name == 'copter':
            log(f'Taking off to {COPTER_TAKEOFF_ALT}m...')
            vehicle.simple_takeoff(COPTER_TAKEOFF_ALT)

            # Wait until altitude is reached
            while True:
                current_alt = vehicle.location.global_relative_frame.alt
                log(f'Copter altitude: {current_alt:.1f}m')

                if current_alt >= COPTER_TAKEOFF_ALT * 0.95:
                    log('Copter reached target altitude')
                    break

                time.sleep(2)

        self.launched_vehicles.add(vehicle_name)
        log(f'{vehicle_name} launched successfully')
        return True

    def _navigate_to(self, vehicle_name, destination):
        """
        Send vehicle to destination using simple_goto

        Args:
            vehicle_name: Name of vehicle
            destination: [lat, lon, alt] list
        """
        vehicle = self.routes[vehicle_name]['vehicle']
        target = LocationGlobalRelative(destination[0], destination[1], destination[2])

        log(f'{vehicle_name} navigating to {destination[0]:.6f}, {destination[1]:.6f}, alt={destination[2]}m')
        vehicle.simple_goto(target)

    def _land_vehicle(self, vehicle_name):
        """
        Land or stop a vehicle

        Args:
            vehicle_name: Name of vehicle
        """
        vehicle = self.routes[vehicle_name]['vehicle']

        if vehicle_name == 'copter':
            log(f'Landing {vehicle_name}...')
            vehicle.mode = VehicleMode('LAND')
        else:
            # For rover/boat, just stop by switching to HOLD mode
            log(f'Stopping {vehicle_name}...')
            vehicle.mode = VehicleMode('HOLD')

    def _all_vehicles_arrived(self):
        """Check if all vehicles have reached their final destinations"""
        all_arrived = True

        for vehicle_name, route in self.routes.items():
            vehicle = route['vehicle']
            destination = route['to']

            if not self._is_arrived(vehicle, destination):
                all_arrived = False
                break

        return all_arrived

    def start(self):
        """Main control loop - coordinates vehicle handoffs"""
        log('========================================')
        log('MULTI-VEHICLE DELIVERY MISSION STARTING')
        log('========================================')
        log(f'Route 1: Rover (Namekawa Station → Opposite Shore Port)')
        log(f'Route 2: Boat (Opposite Shore Port → Main Port)')
        log(f'Route 3: Copter (Main Port → Seven-Eleven)')
        log('========================================')

        mission_start_time = time.time()

        while not self.is_complete:
            log('==== Tower Control Loop ====')

            # Process each vehicle
            for vehicle_name, route in self.routes.items():
                vehicle = route['vehicle']
                wait_for = route['wait_for']
                destination = route['to']
                handoff_point = route['from']

                # Skip if already launched
                if vehicle_name in self.launched_vehicles:
                    # Check if vehicle has arrived at destination
                    if self._is_arrived(vehicle, destination):
                        distance = self._calculate_distance(
                            vehicle,
                            LocationGlobalRelative(destination[0], destination[1], destination[2])
                        )
                        log(f'{vehicle_name} at destination (distance: {distance:.1f}m)')
                    else:
                        # Log current position
                        distance = self._calculate_distance(
                            vehicle,
                            LocationGlobalRelative(destination[0], destination[1], destination[2])
                        )
                        log(f'{vehicle_name} en route (distance to destination: {distance:.1f}m)')

                    continue

                # Check if this vehicle should wait for another
                if wait_for is None:
                    # No dependency, launch immediately
                    log(f'{vehicle_name} has no dependencies, launching...')
                    if self._launch_vehicle(vehicle_name):
                        self._navigate_to(vehicle_name, destination)
                else:
                    # Check if prerequisite vehicle has arrived at handoff point
                    prerequisite_vehicle = self.routes[wait_for]['vehicle']

                    if self._is_arrived(prerequisite_vehicle, handoff_point):
                        distance = self._calculate_distance(
                            prerequisite_vehicle,
                            LocationGlobalRelative(handoff_point[0], handoff_point[1], handoff_point[2])
                        )
                        log(f'{wait_for} arrived at handoff point (distance: {distance:.1f}m)')
                        log(f'Cargo handoff complete: {wait_for} → {vehicle_name}')

                        # Land/stop the prerequisite vehicle
                        self._land_vehicle(wait_for)

                        # Launch this vehicle
                        if self._launch_vehicle(vehicle_name):
                            self._navigate_to(vehicle_name, destination)
                    else:
                        # Still waiting
                        distance = self._calculate_distance(
                            prerequisite_vehicle,
                            LocationGlobalRelative(handoff_point[0], handoff_point[1], handoff_point[2])
                        )
                        log(f'{vehicle_name} waiting for {wait_for} (distance: {distance:.1f}m)')

            # Check if mission is complete
            if self._all_vehicles_arrived():
                log('========================================')
                log('ALL VEHICLES ARRIVED AT DESTINATIONS!')
                log('MISSION COMPLETE')

                mission_duration = time.time() - mission_start_time
                log(f'Total mission time: {mission_duration/60:.1f} minutes')
                log('========================================')

                # Land copter if not already landed
                if self.routes['copter']['vehicle'].mode.name != 'LAND':
                    self._land_vehicle('copter')

                self.is_complete = True
                break

            # Wait before next iteration
            time.sleep(TOWER_LOOP_INTERVAL)

    def cleanup(self):
        """Close all vehicle connections"""
        log('Cleaning up connections...')

        for vehicle_name, route in self.routes.items():
            try:
                route['vehicle'].close()
                log(f'{vehicle_name} connection closed')
            except Exception as e:
                log(f'Error closing {vehicle_name}: {e}')

        log('Cleanup complete')


def main():
    """Main entry point"""
    try:
        log('========================================')
        log('Multi-Vehicle Delivery System')
        log('========================================')

        # Connect to vehicles
        rover, boat, copter = prepare_vehicles()

        # Create control tower
        control_tower = ControlTower(rover, boat, copter)

        # Start mission
        control_tower.start()

        # Cleanup
        control_tower.cleanup()

        log('Program finished successfully')

    except KeyboardInterrupt:
        log('Mission interrupted by user')
    except Exception as e:
        log(f'ERROR: {e}')
        import traceback
        traceback.print_exc()
    finally:
        log('Exiting...')


if __name__ == '__main__':
    main()
