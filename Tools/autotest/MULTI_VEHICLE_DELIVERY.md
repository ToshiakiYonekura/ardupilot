# Multi-Vehicle Autonomous Delivery System

A fully automated multi-vehicle delivery system using ArduPilot SITL simulation. This system coordinates three different vehicle types (Rover, Boat, Copter) operating in sequence with automated cargo handoffs.

## Mission Overview

The system simulates an automated delivery mission across three different transportation modes:

1. **Rover**: Ground vehicle transports cargo from **Namekawa Station** to **Opposite Shore Port**
2. **Boat**: Water vehicle transports cargo from **Opposite Shore Port** to **Main Port**
3. **Copter**: Aerial vehicle delivers cargo from **Main Port** to **Seven-Eleven**

Each vehicle waits for the previous vehicle to arrive at the handoff point before launching, simulating a realistic cargo transfer sequence.

## System Architecture

### Components

- **multi_vehicle_delivery.py**: Central coordinator script that manages all vehicles and handoffs
- **launch_delivery_simulation.sh**: Helper script for launching SITL instances
- **Three SITL instances**: Rover (instance 0), Boat (instance 1), Copter (instance 2)

### Technology Stack

- **ArduPilot SITL**: Software-in-the-loop simulation environment
- **DroneKit Python**: High-level vehicle control API
- **MAVLink**: Communication protocol between vehicles and coordinator
- **Python 3**: Coordinator script language

## Prerequisites

### Required Software

1. **ArduPilot SITL** (already included in this repository)
2. **Python 3.8+**
   ```bash
   python3 --version
   ```

3. **DroneKit Python Library**
   ```bash
   pip3 install dronekit
   ```

4. **pymavlink**
   ```bash
   pip3 install pymavlink
   ```

5. **MAVProxy** (installed with sim_vehicle.py)

### Optional Software

- **tmux** (for automated multi-terminal launch)
  ```bash
  sudo apt-get install tmux
  ```

- **QGroundControl** (for visualization)
  Download from: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

## Installation & Setup

### 1. Build SITL Binaries

```bash
cd /path/to/ardupilot
./waf configure --board sitl
./waf rover copter
```

This builds the necessary binaries:
- `build/sitl/bin/ardurover`
- `build/sitl/bin/arducopter`

### 2. Install Python Dependencies

```bash
pip3 install dronekit pymavlink
```

### 3. Verify Installation

```bash
# Check scripts exist
ls -la Tools/autotest/multi_vehicle_delivery.py
ls -la Tools/autotest/launch_delivery_simulation.sh

# Check they're executable
file Tools/autotest/multi_vehicle_delivery.py
file Tools/autotest/launch_delivery_simulation.sh
```

## Running the Simulation

### Method 1: Manual Launch (Recommended for First-Time Users)

#### Step 1: Launch Rover (Terminal 1)

```bash
cd Rover
../Tools/autotest/sim_vehicle.py -v Rover -I 0 --sysid 1 \
  --out 127.0.0.1:14550 \
  --custom-location=35.876991,140.348026,0,0 \
  --console --map
```

Wait for: `GPS lock at 0 meters` and `armable` message

#### Step 2: Launch Boat (Terminal 2)

```bash
cd Rover
../Tools/autotest/sim_vehicle.py -v Rover -f motorboat -I 1 --sysid 2 \
  --out 127.0.0.1:14560 \
  --custom-location=35.879768,140.348495,0,0 \
  --console --map
```

Wait for: `armable` message

#### Step 3: Launch Copter (Terminal 3)

```bash
cd ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -I 2 --sysid 3 \
  --out 127.0.0.1:14570 \
  --custom-location=35.878275,140.338069,0,0 \
  --console --map
```

Wait for: `armable` message

#### Step 4: Run Coordinator (Terminal 4)

Once all three vehicles show "armable":

```bash
cd /path/to/ardupilot
python3 Tools/autotest/multi_vehicle_delivery.py
```

### Method 2: Using Launch Helper Script

Display launch instructions:
```bash
./Tools/autotest/launch_delivery_simulation.sh
```

### Method 3: Automated tmux Launch (Advanced)

Launch all instances in a tmux session:
```bash
./Tools/autotest/launch_delivery_simulation.sh --tmux
```

Then attach to the session:
```bash
tmux attach -t delivery
```

Navigate to window 3 (coordinator) and run:
```bash
python3 Tools/autotest/multi_vehicle_delivery.py
```

**Tmux keyboard shortcuts:**
- `Ctrl+b` then `0,1,2,3` - Switch to window
- `Ctrl+b` then `n` - Next window
- `Ctrl+b` then `p` - Previous window
- `Ctrl+b` then `d` - Detach from session

## Mission Execution

### Expected Behavior

1. **Phase 1: Rover Launch**
   - Rover arms and navigates from Namekawa Station to Opposite Shore Port
   - Distance: ~370 meters
   - Duration: ~2-3 minutes

2. **Phase 2: Handoff to Boat**
   - When Rover is within 10m of Opposite Shore Port
   - Rover stops (HOLD mode)
   - Boat arms and navigates to Main Port
   - Distance: ~1150 meters
   - Duration: ~4-5 minutes

3. **Phase 3: Handoff to Copter**
   - When Boat is within 10m of Main Port
   - Boat stops (HOLD mode)
   - Copter arms, takes off to 20m altitude
   - Copter navigates to Seven-Eleven at 50m altitude
   - Distance: ~4200 meters
   - Duration: ~6-8 minutes

4. **Mission Complete**
   - All vehicles arrive at destinations
   - Copter lands automatically
   - Mission duration: ~15-20 minutes total

### Coordinator Log Output

The coordinator provides timestamped logs:

```
[2026-01-10 12:00:00] Multi-Vehicle Delivery System
[2026-01-10 12:00:00] Connecting to vehicles...
[2026-01-10 12:00:05] Rover connected
[2026-01-10 12:00:10] Boat connected
[2026-01-10 12:00:15] Copter connected
[2026-01-10 12:00:20] Rover is armable
[2026-01-10 12:00:25] Boat is armable
[2026-01-10 12:00:30] Copter is armable
[2026-01-10 12:00:35] MULTI-VEHICLE DELIVERY MISSION STARTING
[2026-01-10 12:00:35] Launching Rover...
[2026-01-10 12:00:37] Rover navigating to 35.879768, 140.348495
...
[2026-01-10 12:02:45] Rover arrived at handoff point (distance: 8.3m)
[2026-01-10 12:02:45] Cargo handoff complete: rover → boat
[2026-01-10 12:02:45] Launching Boat...
...
[2026-01-10 12:18:20] ALL VEHICLES ARRIVED AT DESTINATIONS!
[2026-01-10 12:18:20] MISSION COMPLETE
[2026-01-10 12:18:20] Total mission time: 17.8 minutes
```

## Configuration

### Adjustable Parameters

Edit `multi_vehicle_delivery.py` to customize:

```python
# Connection ports
ROVER_PORT = '127.0.0.1:14550'
BOAT_PORT = '127.0.0.1:14560'
COPTER_PORT = '127.0.0.1:14570'

# Mission parameters
TOWER_LOOP_INTERVAL = 5  # seconds between status checks
ARRIVAL_THRESHOLD = 10    # meters to consider "arrived"
COPTER_TAKEOFF_ALT = 20   # meters

# Route coordinates
routes = {
    'rover': {
        'from': [35.876991, 140.348026, 0],  # Namekawa Station
        'to': [35.879768, 140.348495, 0],     # Opposite Shore Port
    },
    'boat': {
        'from': [35.879768, 140.348495, 0],   # Opposite Shore Port
        'to': [35.878275, 140.338069, 0],     # Main Port
    },
    'copter': {
        'from': [35.878275, 140.338069, 0],   # Main Port
        'to': [35.877518, 140.295439, 50],    # Seven-Eleven
    }
}
```

## Troubleshooting

### Issue: Vehicle won't arm

**Symptoms:** Vehicle shows "not armable" or arming fails

**Solutions:**
- Wait for GPS lock (you'll see "GPS lock at 0 meters")
- Check MAVProxy console for errors
- Ensure vehicle is in GUIDED mode: `mode GUIDED`
- Check EKF status: `ekf status`

### Issue: DroneKit connection timeout

**Symptoms:** `APIException: Timeout in connect`

**Solutions:**
- Verify SITL instance is running
- Check port numbers match (14550, 14560, 14570)
- Wait longer for SITL to fully start (30+ seconds)
- Try increasing timeout in `connect()` call

### Issue: Vehicle doesn't move

**Symptoms:** Vehicle arms but stays in place

**Solutions:**
- Check vehicle is in GUIDED mode
- Verify coordinates are valid (not NaN)
- Check MAVProxy console for navigation errors
- Ensure GPS lock exists

### Issue: Handoff doesn't occur

**Symptoms:** Next vehicle never launches

**Solutions:**
- Check distance threshold (default 10m)
- Verify prerequisite vehicle is moving toward handoff point
- Review coordinator logs for arrival detection
- Increase `ARRIVAL_THRESHOLD` if needed

### Issue: Copter won't takeoff

**Symptoms:** Copter arms but doesn't takeoff

**Solutions:**
- Check altitude reading in MAVProxy: `status`
- Ensure battery voltage is sufficient
- Verify no fence violations
- Check for arming checks: `arm check`

## Monitoring & Debugging

### MAVProxy Console Commands

Monitor individual vehicles in their MAVProxy consoles:

```bash
# Check current mode
mode

# Check position
status

# Check GPS
gps

# Check EKF
ekf status

# Check mission progress (if using AUTO mode)
wp list

# Manually change mode
mode GUIDED
mode LAND
mode RTL

# Manual arming
arm throttle
disarm
```

### Coordinator Debug Mode

Add verbose logging to `multi_vehicle_delivery.py`:

```python
# In _calculate_distance method
print(f"Vehicle: {vehicle.vehicle_name}")
print(f"Current: {vehicle.location.global_frame.lat}, {vehicle.location.global_frame.lon}")
print(f"Target: {target_location.lat}, {target_location.lon}")
print(f"Distance: {distance:.2f}m")
```

### QGroundControl Visualization

1. Open QGroundControl
2. It should automatically connect to UDP:14550 (Rover)
3. To connect to multiple vehicles:
   - Application Settings → Comm Links
   - Add new UDP connection for each port (14550, 14560, 14570)

## Technical Details

### Port Allocation

| Vehicle | Instance | SYSID | TCP Port | UDP Port (DroneKit) |
|---------|----------|-------|----------|-------------------|
| Rover   | 0        | 1     | 5760     | 14550            |
| Boat    | 1        | 2     | 5770     | 14560            |
| Copter  | 2        | 3     | 5780     | 14570            |

### Distance Calculation

The system uses haversine approximation for distance:

```python
dlat = vehicle.lat - target.lat
dlong = vehicle.lon - target.lon
distance = sqrt((dlat² + dlong²)) * 111,319.5 meters
```

This is accurate for distances < 100km.

### Coordinate System

- **Format**: WGS84 decimal degrees
- **Latitude**: Positive = North, Negative = South
- **Longitude**: Positive = East, Negative = West
- **Altitude**: Meters above ground level (AGL)

### Handoff Detection

A vehicle is considered "arrived" when:
1. Distance to target < `ARRIVAL_THRESHOLD` (10m default)
2. Position is updated (vehicle is not idle)

## Extending the System

### Adding Intermediate Waypoints

Modify routes to include waypoints:

```python
routes = {
    'rover': {
        'vehicle': None,
        'from': [35.876991, 140.348026, 0],
        'waypoints': [
            [35.877500, 140.348200, 0],  # Intermediate point 1
            [35.878500, 140.348350, 0],  # Intermediate point 2
        ],
        'to': [35.879768, 140.348495, 0],
        'wait_for': None
    }
}
```

Then update `_navigate_to()` to handle waypoint lists.

### Adding Cargo Transfer Delay

Add delay after handoff:

```python
log(f'Cargo transfer in progress...')
time.sleep(30)  # 30 second transfer time
log(f'Cargo transfer complete')
```

### Adding Battery Monitoring

Monitor battery and implement low-battery RTL:

```python
def _check_battery(self, vehicle_name):
    vehicle = self.routes[vehicle_name]['vehicle']
    if vehicle.battery.level < 20:
        log(f'WARNING: {vehicle_name} low battery ({vehicle.battery.level}%)')
        vehicle.mode = VehicleMode('RTL')
```

### Adding Geofencing

Define safety boundaries:

```python
def _check_geofence(self, vehicle):
    lat = vehicle.location.global_frame.lat
    lon = vehicle.location.global_frame.lon

    # Define fence boundaries
    if not (35.870 < lat < 35.885 and 140.290 < lon < 140.355):
        log(f'WARNING: {vehicle.vehicle_name} outside geofence!')
        return False
    return True
```

## Performance Metrics

### Expected Performance

- **Rover speed**: ~2-3 m/s (configurable with `WP_SPEED` parameter)
- **Boat speed**: ~4-5 m/s (configurable with `WP_SPEED` parameter)
- **Copter speed**: ~10-12 m/s (configurable with `WPNAV_SPEED` parameter)

### Mission Statistics

- **Total distance**: ~5,720 meters
- **Handoff points**: 2 (Opposite Shore Port, Main Port)
- **Altitude range**: 0-50 meters
- **Estimated time**: 15-20 minutes (SITL speed)

## Safety Considerations

### SITL Simulation Only

**IMPORTANT:** This system is designed for SITL simulation only. Before adapting for real hardware:

1. Add comprehensive error handling
2. Implement geofencing
3. Add battery monitoring and failsafes
4. Test extensively in controlled environments
5. Follow local aviation regulations (for copter)
6. Obtain necessary permits and clearances

### Emergency Stop

Press `Ctrl+C` in coordinator terminal to stop mission. Then manually land/disarm vehicles:

```bash
# In each MAVProxy console
mode LAND    # or RTL
disarm
```

## References

- ArduPilot Documentation: https://ardupilot.org/
- DroneKit Python: https://dronekit-python.readthedocs.io/
- MAVLink Protocol: https://mavlink.io/
- SITL Simulator: https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html

## Support

For issues or questions:
- ArduPilot Forum: https://discuss.ardupilot.org/
- DroneKit Forum: https://github.com/dronekit/dronekit-python/issues
- ArduPilot Discord: https://ardupilot.org/discord

## License

This system is released under the same license as ArduPilot (GPLv3).

## Authors

Multi-Vehicle Delivery System - ArduPilot Project

---

**Last Updated**: 2026-01-10
