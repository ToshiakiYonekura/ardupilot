# Mode 99 - LQR State Feedback Control

## Overview

Mode 99 (SMART_PHOTO) is a custom ArduCopter flight mode that implements LQR (Linear Quadratic Regulator) momentum-based state feedback control. It is designed to work with a Raspberry Pi companion computer that handles all mission planning and state machine logic.

## Architecture

### Division of Responsibilities

**ArduPilot (Mode 99):**
- Execute LQR state feedback control at 100Hz
- Monitor safety conditions (battery, GPS, EKF, heartbeat)
- Automatically transition to LAND mode on critical failures
- Provide telemetry to companion computer

**Raspberry Pi (Companion Computer):**
- Manage all state transitions (arming, takeoff, mission, landing)
- Generate position/velocity commands at 20Hz
- Plan trajectories and waypoints
- Handle obstacle avoidance
- Monitor system health

## Implementation Files

- `ArduCopter/mode_smartphoto99.h` - Mode class definition
- `ArduCopter/mode_smartphoto99.cpp` - Implementation
- `ArduCopter/GCS_MAVLink_Copter.cpp` - MAVLink command handling

## Control System

### LQR State Feedback

**State Vector (12 states):**
```
x = [pos_n, pos_e, pos_d, vel_n, vel_e, vel_d, roll, pitch, yaw, p, q, r]
```

**Control Vector (4 outputs):**
```
u = [F_thrust, M_roll, M_pitch, M_yaw]
```

**Control Law:**
```
u = u_hover - K * (x - x_ref)
```

Where:
- `K` is the 4×12 gain matrix computed from system identification
- `x` is the current state from EKF
- `x_ref` is the reference state from companion commands
- `u_hover` is the equilibrium control (hover thrust, zero moments)

### Timing

- **Main loop:** 400Hz (ArduPilot standard)
- **LQR control:** 100Hz (computational efficiency)
- **Wind telemetry:** 100Hz
- **Failsafe checks:** Every cycle (400Hz)
- **Expected commands:** 20Hz from companion

## MAVLink Interface

### Commands to Mode 99

**SET_POSITION_TARGET_LOCAL_NED** (Expected at 20Hz)
```
Position: (pos_n, pos_e, pos_d) [meters, NED frame]
Velocity: (vel_n, vel_e, vel_d) [m/s, NED frame]
Yaw: yaw_target [radians, 0 = North]
Yaw Rate: yaw_rate_target [rad/s]
```

**HEARTBEAT** (Minimum 1Hz)
- Required to prevent communication loss failsafe
- Automatically sent by companion on each command

### Telemetry from Mode 99

- Wind estimate (3D vector, NED frame) @ 100Hz
- Standard telemetry: GLOBAL_POSITION_INT, LOCAL_POSITION_NED, ATTITUDE, etc.
- EKF_STATUS_REPORT for health monitoring
- Battery status via SYS_STATUS

## Failsafe System

Mode 99 implements four critical failsafes. All trigger automatic transition to LAND mode:

### 1. Communication Loss Failsafe
- **Monitor:** Heartbeat from Raspberry Pi
- **Timeout:** 1 second
- **Action:** Transition to LAND mode
- **Reason:** ModeReason::RADIO_FAILSAFE

### 2. Battery Critical Failsafe
- **Monitor:** Battery percentage
- **Threshold:** 20% (critical), 30% (warning)
- **Action:** Transition to LAND mode
- **Reason:** ModeReason::BATTERY_FAILSAFE

### 3. EKF Instability Failsafe
- **Monitor:** EKF health flags and innovation values
- **Condition:** EKF unhealthy
- **Action:** Transition to LAND mode
- **Reason:** ModeReason::EKF_FAILSAFE

### 4. GPS Health Failsafe
- **Monitor:** GPS satellite count and HDOP
- **Requirements:** ≥10 satellites, HDOP ≤1.5
- **Action:** Transition to LAND mode
- **Reason:** ModeReason::GPS_GLITCH

## Coordinate System

All positions, velocities, and commands use the **NED (North-East-Down)** coordinate system:

- **Origin:** Set at arm position
- **Axes:**
  - X = North (positive forward)
  - Y = East (positive right)
  - Z = Down (positive downward)
- **Units:**
  - Position: meters
  - Velocity: m/s
  - Angles: radians
  - Angular rates: rad/s

**Important:** All MAVLink commands MUST use these exact units and frame.

## System Identification

Mode 99 requires system identification parameters from Mode 98. These are stored in `sysid_params.txt`:

### Required Parameters

```
MASS=<mass_kg>              # Vehicle mass in kg
IXX=<Ixx>                   # Roll moment of inertia
IYY=<Iyy>                   # Pitch moment of inertia
IZZ=<Izz>                   # Yaw moment of inertia
MOTOR_KV=<kv>              # Motor KV rating
MAX_THRUST=<thrust_N>       # Max thrust per motor (Newtons)
ROLL_GAIN=<gain>           # Identified roll rate gain
PITCH_GAIN=<gain>          # Identified pitch rate gain
YAW_GAIN=<gain>            # Identified yaw rate gain
THROTTLE_HOVER=<hover>     # Hover throttle (0-1)
SAMPLES=<count>            # Number of samples used
```

### LQR Gain Calculation

The LQR gain matrix K is computed using:
- **Q matrix:** State cost weights (position, velocity, attitude, rates)
- **R matrix:** Control cost weights (thrust, moments)
- Simplified LQR approach suitable for embedded implementation
- Gains tuned for safety and stability margins

## Usage

### Building

```bash
# Configure for SITL
./waf configure --board sitl

# Build ArduCopter
./waf copter

# Binary will be at: build/sitl/bin/arducopter
```

### Running in SITL

```bash
# Start SITL
cd ArduCopter
../Tools/autotest/sim_vehicle.py --console --map

# In MAVProxy console:
mode SMART_PHOTO    # Enter Mode 99
```

### Entry Requirements

- Valid position estimate from EKF
- System ID parameters file exists
- Can be entered from any mode (typically from STABILIZE or LOITER)

### Exit Behavior

**Normal Exit:**
- User command to change mode (e.g., `mode LOITER`)
- Returns control to standard ArduPilot controllers

**Emergency Exit:**
- Automatic transition to LAND mode on any failsafe trigger
- LAND mode will use standard ArduPilot landing logic

## Companion Computer Implementation

The companion computer (Raspberry Pi) is responsible for:

### State Machine Management

1. **Pre-flight:**
   - Verify battery level (≥90%)
   - Check GPS lock and EKF health
   - Wait for operator approval

2. **Arming:**
   - Send ARM command via MAVLink
   - Wait for armed confirmation

3. **Takeoff:**
   - Send ascending position commands
   - Monitor altitude until target reached (e.g., 50m)

4. **Mission Execution:**
   - Generate waypoint trajectories
   - Send position/velocity commands at 20Hz
   - Monitor progress and obstacles

5. **Landing:**
   - Send descending position commands
   - Reduce descent rate near ground
   - Monitor landing detection

6. **Post-flight:**
   - Send DISARM command
   - Log mission data

### Command Generation (20Hz)

```python
# Example pseudocode
def generate_command(current_state, waypoint):
    # Calculate position target
    pos_ned = calculate_position_target(current_state, waypoint)

    # Calculate velocity target
    vel_ned = calculate_velocity_command(current_state, waypoint)

    # Calculate yaw target
    yaw = calculate_heading(current_state, waypoint)
    yaw_rate = calculate_yaw_rate(current_state, waypoint)

    # Send to Mode 99
    send_set_position_target_local_ned(
        pos_ned[0], pos_ned[1], pos_ned[2],
        vel_ned[0], vel_ned[1], vel_ned[2],
        yaw, yaw_rate
    )
```

### Safety Monitoring

The companion should monitor:
- EKF_STATUS_REPORT messages
- Battery status via SYS_STATUS
- GPS health
- Wind speed warnings
- Mode changes (detect failsafe triggers)

## Important Notes

1. **No State Machine in ArduPilot:** All mission logic is handled by the companion computer.

2. **Continuous Control:** Mode 99 continuously executes LQR control at 100Hz regardless of mission phase.

3. **Heartbeat Critical:** The companion must send commands or heartbeats at least every 1 second to prevent failsafe.

4. **Units Matter:** All commands must use SI units (meters, m/s, radians, rad/s). NO centimeters or degrees.

5. **NED Frame:** All commands and states are in NED coordinate system with origin at arm position.

6. **Failsafes are Automatic:** ArduPilot will automatically handle critical failures by transitioning to LAND mode.

7. **System ID Required:** Mode 99 requires system identification parameters from Mode 98 before first use.

8. **Wind Data:** Mode 99 provides 3D wind estimates at 100Hz for trajectory planning.

## Troubleshooting

### Mode won't enter
- Check EKF health: `ekf_status`
- Verify position estimate: `position_ok()`
- Ensure GPS lock with ≥10 satellites

### Unexpected LAND mode
- Check for failsafe messages in MAVLink
- Verify heartbeat is being sent (≥1Hz)
- Check battery level (>20%)
- Verify EKF and GPS health

### Poor tracking performance
- Check LQR gains calculation
- Verify system ID parameters are loaded
- Monitor state feedback telemetry
- Check wind conditions

### Command not responding
- Verify 20Hz command rate
- Check NED coordinate frame
- Verify units (meters, m/s, radians)
- Ensure commands are within safety limits

## References

- ArduPilot documentation: https://ardupilot.org
- MAVLink protocol: https://mavlink.io
- LQR control theory: Standard optimal control textbooks
- System identification: Mode 98 implementation
