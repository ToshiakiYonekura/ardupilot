# Running Multi-Vehicle Delivery with Mission Planner

This guide explains how to visualize the multi-vehicle autonomous delivery mission using Mission Planner while the coordinator script manages the automation.

## Overview

The setup uses dual UDP outputs for each vehicle:
- **Port 14550/14560/14570**: Used by DroneKit coordinator script
- **Port 14551/14561/14571**: Used by Mission Planner for visualization

This allows both systems to work simultaneously without conflicts.

## Prerequisites

### Option 1: Mission Planner (Windows/Linux with Mono)

**Windows:**
- Download from: https://firmware.ardupilot.org/Tools/MissionPlanner/
- Install and run MissionPlanner.exe

**Linux with Mono:**
```bash
# Install Mono
sudo apt install mono-complete

# Download Mission Planner
wget https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.zip
unzip MissionPlanner-latest.zip
mono MissionPlanner.exe
```

### Option 2: QGroundControl (Recommended for Multi-Vehicle)

QGroundControl has better native support for multiple vehicles.

**Linux:**
```bash
# Download AppImage
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
./QGroundControl.AppImage
```

**Windows/Mac:**
- Download from: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

## Quick Start

### Interactive Setup Script

Run the Mission Planner setup script:
```bash
./Tools/autotest/launch_delivery_missionplanner.sh
```

Select option 4 for complete setup, or individual options:
- Option 1: Show manual launch commands
- Option 2: Mission Planner connection guide
- Option 3: Create automated start script
- Option 4: All of the above

## Manual Setup (Step-by-Step)

### Step 1: Launch SITL Instances

Open 3 separate terminals and run these commands:

**Terminal 1 - Rover:**
```bash
cd Rover
../Tools/autotest/sim_vehicle.py -v Rover -I 0 --sysid 1 \
  --out 127.0.0.1:14550 --out 127.0.0.1:14551 \
  --custom-location=35.876991,140.348026,0,0 \
  --console --map
```

**Terminal 2 - Boat:**
```bash
cd Rover
../Tools/autotest/sim_vehicle.py -v Rover -f motorboat -I 1 --sysid 2 \
  --out 127.0.0.1:14560 --out 127.0.0.1:14561 \
  --custom-location=35.879768,140.348495,0,0 \
  --console --map
```

**Terminal 3 - Copter:**
```bash
cd ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -I 2 --sysid 3 \
  --out 127.0.0.1:14570 --out 127.0.0.1:14571 \
  --custom-location=35.878275,140.338069,0,0 \
  --console --map
```

Wait for all vehicles to show "armable" status.

### Step 2: Connect Mission Planner

#### Single Vehicle View

1. Open Mission Planner
2. In the top-right corner, select **UDP** from the connection dropdown
3. Click the **Connect** button
4. A dialog will appear asking for the port number
5. Enter one of these ports:
   - **14551** for Rover
   - **14561** for Boat
   - **14571** for Copter
6. Click **OK**

Mission Planner will connect and display the selected vehicle on the map.

**To switch between vehicles:**
- Click **Disconnect**
- Click **Connect** again
- Enter a different port number

#### Multiple Mission Planner Windows (Advanced)

You can run multiple instances of Mission Planner to view all vehicles:

1. Open Mission Planner 3 times (3 separate windows)
2. Connect each to different ports:
   - Window 1: Port 14551 (Rover)
   - Window 2: Port 14561 (Boat)
   - Window 3: Port 14571 (Copter)
3. Arrange windows to view all simultaneously

### Step 3: Run Coordinator Script

In a 4th terminal:
```bash
cd /path/to/ardupilot
python3 Tools/autotest/multi_vehicle_delivery.py
```

The coordinator will:
1. Connect to all three vehicles (via ports 14550, 14560, 14570)
2. Launch them in sequence with handoffs
3. Monitor progress and log status

You'll see the vehicles move in Mission Planner as the coordinator controls them.

## Using QGroundControl (Recommended)

QGroundControl has superior multi-vehicle support:

### Step 1: Configure Communication Links

1. Launch QGroundControl
2. Click the **Q icon** (top-left) → **Application Settings**
3. Select **Comm Links** from the left menu
4. Add three UDP connections:

**Link 1 - Rover:**
- Click **Add** button
- Type: **UDP**
- Name: **Rover**
- Listening Port: **14551**
- Server Addresses: (leave empty for listening mode)
- Click **OK**

**Link 2 - Boat:**
- Click **Add** button
- Type: **UDP**
- Name: **Boat**
- Listening Port: **14561**
- Click **OK**

**Link 3 - Copter:**
- Click **Add** button
- Type: **UDP**
- Name: **Copter**
- Listening Port: **14571**
- Click **OK**

### Step 2: Connect to Vehicles

1. Click **Connect** for each link
2. All vehicles will appear in the vehicle selector (top toolbar)
3. Click the vehicle dropdown to switch between vehicles
4. The map shows all connected vehicles simultaneously

### Step 3: Run Mission

Launch SITL instances and coordinator script as described above. QGC will show all vehicles in real-time.

## Automated Launch with Screen

The setup script can create an automated launcher:

```bash
./Tools/autotest/launch_delivery_missionplanner.sh
# Select option 3
```

This creates `start_delivery_mission.sh` which:
1. Launches all SITL instances in screen sessions
2. Waits for initialization
3. Runs the coordinator script
4. Provides commands to stop everything

Run it with:
```bash
./start_delivery_mission.sh
```

View individual vehicle consoles:
```bash
screen -r delivery_rover
screen -r delivery_boat
screen -r delivery_copter
```

Press `Ctrl+A` then `D` to detach from screen session.

Stop all vehicles:
```bash
screen -S delivery_rover -X quit
screen -S delivery_boat -X quit
screen -S delivery_copter -X quit
```

## Monitoring the Mission

### In Mission Planner

**Flight Data Screen:**
- Current altitude, speed, battery level
- GPS status and satellite count
- Mode (GUIDED, LAND, HOLD)
- Armed/disarmed status

**Flight Plan Screen:**
- Vehicle position on map
- Home location marker
- Current heading

**HUD (Heads-Up Display):**
- Artificial horizon
- Altitude and speed indicators
- Heading compass

### In MAVProxy Console

Each SITL terminal shows MAVProxy console commands:

```bash
# Check mode
mode

# Check position
status

# Check mission progress
wp list

# Get distance to waypoint
distance

# Monitor telemetry
watch GLOBAL_POSITION_INT
```

### In Coordinator Logs

Terminal 4 shows detailed mission progress:
```
[2026-01-10 12:00:35] Launching Rover...
[2026-01-10 12:00:37] Rover navigating to 35.879768, 140.348495
[2026-01-10 12:02:45] Rover arrived at handoff point (distance: 8.3m)
[2026-01-10 12:02:45] Cargo handoff complete: rover → boat
```

## Viewing Mission in Real-Time

### Mission Planner Map View

1. Go to **Flight Data** screen
2. Right-click on map for options:
   - Zoom in/out
   - Set home position
   - Add waypoints (manual mode)
   - Clear tracks

3. Use **Fly to Here** (right-click) for manual override (not recommended during automated mission)

### Setting Map Type

Click **Map Type** dropdown:
- Google Maps
- Google Hybrid
- Google Terrain
- Bing Maps
- OpenStreetMap

### Tracking Vehicle

Mission Planner automatically follows the active vehicle. To disable:
- Right-click map → **Auto Pan: Off**

## Troubleshooting

### Mission Planner won't connect

**Check:**
- Correct port number (14551, 14561, or 14571)
- SITL instance is running
- No firewall blocking UDP ports
- Using UDP, not TCP

**Solution:**
```bash
# Verify SITL is sending data
netstat -an | grep 14551
# Should show UDP listening on that port
```

### Can't see vehicle on map

**Check:**
- GPS lock achieved (wait 30 seconds after SITL start)
- Location coordinates valid
- Map downloaded (requires internet)

**Solution in MAVProxy console:**
```bash
# Check GPS status
gps

# Check position
status

# Should show latitude/longitude
```

### Multiple vehicles show same position

**Check:**
- Each vehicle has unique SYSID (--sysid 1, 2, 3)
- Connected to correct port for each vehicle

**Solution:**
Verify SYSID in MAVProxy:
```bash
param show SYSID_THISMAV
# Should be 1 for rover, 2 for boat, 3 for copter
```

### Vehicle not responding to coordinator

**Check:**
- Coordinator connected to ports 14550/14560/14570 (not 14551/14561/14571)
- Vehicle is armable
- No mode conflicts

**Solution:**
Check coordinator logs for connection messages:
```
[2026-01-10 12:00:05] Rover connected
[2026-01-10 12:00:20] Rover is armable
```

## Port Reference Table

| Vehicle | SYSID | DroneKit Port | Mission Planner Port | MAVProxy TCP |
|---------|-------|---------------|---------------------|--------------|
| Rover   | 1     | UDP 14550     | UDP 14551          | TCP 5760     |
| Boat    | 2     | UDP 14560     | UDP 14561          | TCP 5770     |
| Copter  | 3     | UDP 14570     | UDP 14571          | TCP 5780     |

## Advanced: Recording Mission

### Mission Planner Telemetry Logs

Mission Planner automatically records telemetry logs (.tlog files):
- Location: `Documents\Mission Planner\logs\`
- Format: `YYYY-MM-DD HH-MM-SS.tlog`

**Replay a log:**
1. Go to **Flight Data** screen
2. Click **Telemetry Logs** button
3. Select log file
4. Click **Load Log**
5. Use playback controls to replay mission

### DataFlash Logs

SITL also creates DataFlash logs:
- Location: `logs/` directory in vehicle folder
- Format: `.bin` files

**Analyze with Mission Planner:**
1. Go to **Flight Data** screen
2. Click **DataFlash Logs** button
3. Click **Review a Log**
4. Select `.bin` file
5. View graphs of all parameters

## Mission Planner Features During Flight

### Real-Time Parameter Changes

1. Go to **Config/Tuning** → **Full Parameter List**
2. Search for parameter (e.g., `WP_SPEED`)
3. Change value
4. Click **Write Params**

**Warning:** Changing parameters during mission may affect behavior.

### Manual Override

If you need to take manual control:
1. Click **Actions** tab in Flight Data
2. Click **Change Mode** → **GUIDED**
3. Right-click map → **Fly to Here**

**Note:** This will interrupt the coordinator script's automation.

### Emergency Actions

In Flight Data screen, **Actions** tab:
- **RTL** - Return to launch
- **Land** - Land immediately
- **Arm/Disarm** - Emergency disarm
- **Set Mode** - Change flight mode

## Best Practices

1. **Always start Mission Planner before coordinator script**
   - Allows you to monitor connection and armability
   - Ensures vehicles are ready

2. **Use QGroundControl for multi-vehicle missions**
   - Easier to monitor all three vehicles
   - Native multi-vehicle support

3. **Keep MAVProxy consoles visible**
   - Shows errors and warnings immediately
   - Provides manual control option

4. **Monitor coordinator terminal**
   - Shows mission progress and handoffs
   - First place to check for issues

5. **Don't manually control vehicles during automation**
   - Let coordinator script manage mission
   - Manual override can break handoff logic

## Further Reading

- Mission Planner Documentation: https://ardupilot.org/planner/
- QGroundControl User Guide: https://docs.qgroundcontrol.com/
- ArduPilot SITL: https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
- MAVLink Protocol: https://mavlink.io/

## Summary

The multi-vehicle delivery system works seamlessly with Mission Planner by using dual UDP outputs. The coordinator script handles all automation via DroneKit while Mission Planner provides real-time visualization and monitoring on separate ports. This architecture ensures no conflicts while giving you full visibility into the mission progress.

---

**Last Updated**: 2026-01-10
