# Quick Start - Multi-Vehicle Delivery with Mission Planner

## Simple 4-Terminal Setup

Copy and paste these commands into 4 separate terminal windows:

### Terminal 1 - Rover
```bash
cd ~/ardupilot/Rover && ../Tools/autotest/sim_vehicle.py -v Rover -I 0 --sysid 1 --custom-location=35.876991,140.348026,0,0 --console --map
```

### Terminal 2 - Boat
```bash
cd ~/ardupilot/Rover && ../Tools/autotest/sim_vehicle.py -v Rover -f motorboat -I 1 --sysid 2 --custom-location=35.879768,140.348495,0,0 --console --map
```

### Terminal 3 - Copter
```bash
cd ~/ardupilot/ArduCopter && ../Tools/autotest/sim_vehicle.py -v ArduCopter -I 2 --sysid 3 --custom-location=35.878275,140.338069,0,0 --console --map
```

Wait for all 3 to show "armable" (30-60 seconds)

### Terminal 4 - Coordinator
```bash
cd ~/ardupilot && python3 Tools/autotest/multi_vehicle_delivery.py
```

## Mission Planner Connection

1. Open Mission Planner
2. Select "TCP" from dropdown (top-right)
3. Click "Connect"
4. Enter port:
   - **5760** for Rover
   - **5770** for Boat
   - **5780** for Copter

## What You'll See

- **Rover** launches first, drives to Opposite Shore Port
- **Boat** launches when Rover arrives, sails to Main Port
- **Copter** launches when Boat arrives, flies to Seven-Eleven
- Mission complete when all vehicles arrive!

## Ports Reference

| Vehicle | TCP Port (Mission Planner) | TCP Port (DroneKit) |
|---------|---------------------------|---------------------|
| Rover   | 5760                      | 5760               |
| Boat    | 5770                      | 5770               |
| Copter  | 5780                      | 5780               |
