# Hardware Testing Procedures - Mode 99 LQR Controller

## Overview

This document provides comprehensive procedures for testing the LQR controller on actual hardware (flight controller + copter).

⚠️ **SAFETY FIRST** - Always follow safety protocols when testing on hardware

---

## Pre-Flight Checklist

### Hardware Requirements

- [ ] ArduPilot-compatible flight controller (Pixhawk, Cube, etc.)
- [ ] Quadcopter with known mass and inertia
- [ ] GPS module (for position estimation)
- [ ] Telemetry radio (for monitoring)
- [ ] Ground control station (Mission Planner / QGroundControl)
- [ ] RC transmitter with at least 6 channels
- [ ] Fully charged batteries
- [ ] Props removed for initial testing

### Software Requirements

- [ ] ArduCopter with Mode 99 compiled
- [ ] System ID parameters file (`sysid_params.txt`) on SD card
- [ ] Ground control software installed
- [ ] Telemetry logging enabled
- [ ] Emergency procedures reviewed

---

## Phase 1: Bench Testing (Props OFF)

### 1.1 Flash Firmware

```bash
cd ~/ardupilot

# Configure for your board
./waf configure --board CubeBlack  # Or your board type
./waf list_boards  # See all available boards

# Build
./waf copter

# Upload (connect flight controller via USB)
./waf copter --upload
```

**Expected output:**
```
...
Upload successful
```

### 1.2 Copy System ID Parameters

**Option A: SD Card**
1. Remove SD card from flight controller
2. Copy `sysid_params.txt` to root of SD card
3. Reinsert SD card

**Option B: MAVLink File Transfer**
```bash
# Using MAVProxy
mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600

MAV> ftp put sysid_params.txt sysid_params.txt
```

### 1.3 Configure Parameters

Connect via ground control station and set:

**Essential Parameters:**
```
AHRS_EKF_TYPE = 3          # Use EKF3
EK3_ENABLE = 1             # Enable EKF3
EK3_SRC1_POSXY = 3         # GPS for position
EK3_SRC1_POSZ = 1          # Baro for altitude
EK3_SRC1_VELXY = 3         # GPS for velocity
EK3_SRC1_VELZ = 3          # GPS for vertical velocity
GPS_TYPE = 1               # Auto detect GPS

# Flight mode assignments
FLTMODE1 = 0               # Stabilize (emergency)
FLTMODE2 = 2               # Alt Hold
FLTMODE3 = 5               # Loiter
FLTMODE4 = 99              # Mode 99 (SMARTPH99)
FLTMODE5 = 6               # RTL
FLTMODE6 = 9               # Land

# Failsafe
FS_THR_ENABLE = 1          # Enable throttle failsafe
FS_THR_VALUE = 975         # Failsafe PWM threshold
FS_GCS_ENABLE = 1          # GCS failsafe
```

**Tuning Parameters (adjust for your copter):**
```
MOT_THST_HOVER = 0.35      # Hover throttle (adjust)
PSC_POSXY_P = 1.0          # Position control P gain
PSC_VELXY_P = 2.0          # Velocity control P gain
```

### 1.4 Pre-Arm Checks (Props OFF)

1. **Power on** flight controller
2. **Connect** telemetry
3. **Check GPS lock**: Wait for 10+ satellites
4. **Check EKF status**: Should show "EKF OK"
5. **Check sensors**: IMU, Compass, Baro all green
6. **Calibrate** if needed:
   - Accelerometer calibration
   - Compass calibration
   - Radio calibration

### 1.5 Mode 99 Initialization Test

1. **Arm** the vehicle (props OFF!)
   - Throttle down + yaw right for 2 seconds
   - Or use "Arm" button in ground control

2. **Switch to Mode 99** (SMARTPH99)
   - Use flight mode switch on transmitter
   - Or command from ground control

3. **Watch for messages** in ground control:
   ```
   SMARTPHOTO99: Using LQR momentum-based state feedback
   SMARTPHOTO99: LQR gains calculated
   Mass=X.XX kg, Hover=XX.X N
   SMARTPHOTO99: Entered PLANNING state
   ```

4. **Check telemetry**:
   - Look for `LQR_Thrust` value (~mass × 9.81 N)
   - Look for `LQR_Rate` value (100.0 Hz)
   - Moments should be near zero

5. **Disarm** the vehicle

**✅ Pass criteria:**
- Mode 99 activates successfully
- LQR initialization messages appear
- LQR gains calculated correctly
- Telemetry streams at 100Hz

---

## Phase 2: Ground Testing (Props ON, Tethered)

⚠️ **SAFETY**: Props ON, copter secured/tethered

### 2.1 Setup

1. **Install props** (correct rotation!)
2. **Secure copter**:
   - Option A: Tether with rope (test stand)
   - Option B: Hold firmly on ground
   - Option C: Indoor test cage
3. **Clear area** of people and objects
4. **Emergency procedures** ready:
   - Kill switch on transmitter
   - Disarm switch ready
   - Ground control "Emergency Stop"

### 2.2 Motor Response Test

1. **Arm** in Stabilize mode
2. **Slowly increase throttle** to ~30%
3. **Check motor response**:
   - All motors spinning
   - No excessive vibration
   - Correct rotation direction
4. **Test attitude commands**:
   - Roll stick → roll motors respond
   - Pitch stick → pitch motors respond
   - Yaw stick → yaw motors respond
5. **Disarm**

### 2.3 Mode 99 LQR Response Test

1. **Arm** in Stabilize mode
2. **Switch to Mode 99**
3. **Watch initialization messages**
4. **Slowly increase throttle** to hover point (~50%)
5. **Observe LQR telemetry**:
   - `LQR_Thrust` should increase with throttle
   - `LQR_M_roll`, `LQR_M_pitch` react to tilts
   - Attitude commands try to level copter
6. **Test stick inputs**:
   - Roll/Pitch: Should command attitude changes
   - Yaw: Should command yaw rate
   - Throttle: Should adjust altitude target
7. **Disarm**

**✅ Pass criteria:**
- Mode 99 responds to inputs
- LQR telemetry shows reasonable values
- Motors respond to LQR commands
- No oscillations or instability

---

## Phase 3: Hover Test (First Flight)

⚠️ **CRITICAL SAFETY**
- Open area, no obstacles
- Calm weather (< 5 m/s wind)
- Experienced pilot ready
- Emergency procedures briefed

### 3.1 Pre-Flight

1. **Weather check**: Calm, clear
2. **Area clear**: 30m radius minimum
3. **Battery check**: Fully charged
4. **GPS check**: 10+ satellites, HDOP < 1.5
5. **Preflight checks**: All systems green
6. **Emergency plan**: Know how to disarm, switch modes

### 3.2 Takeoff in Stabilize Mode

1. **Arm** in Stabilize mode (NOT Mode 99 yet!)
2. **Gentle throttle increase** to 50-60%
3. **Lift off** 1-2 meters
4. **Hover** for 30 seconds:
   - Check stability
   - Check control response
   - Check battery usage
   - Check telemetry
5. **Land gently**
6. **Disarm**

**If any issues** → DO NOT proceed to Mode 99

### 3.3 Mode 99 LQR Hover Test

1. **Arm** in Stabilize mode
2. **Takeoff** to 2m altitude
3. **Stabilize** hover
4. **Switch to Mode 99**
5. **Observe immediately**:
   - LQR initialization messages
   - Copter behavior (should stabilize)
   - Telemetry values
6. **Hover** for 30 seconds:
   - Minimal stick inputs
   - Let LQR controller work
   - Monitor telemetry
7. **Check performance**:
   - Altitude hold accuracy (± 0.5m)
   - Position hold accuracy (± 1m)
   - Attitude stability (± 5°)
   - Control smoothness
8. **Switch back to Stabilize**
9. **Land**
10. **Disarm**

**✅ Pass criteria:**
- Smooth transition to Mode 99
- Stable hover achieved
- LQR telemetry normal
- No oscillations
- Good control response

**❌ Abort if:**
- Oscillations appear
- Copter unstable
- Telemetry abnormal
- Unusual sounds/vibrations

---

## Phase 4: Position Control Test

### 4.1 Basic Position Hold

1. **Takeoff** to 5m in Stabilize
2. **Switch to Mode 99**
3. **Release sticks** (center position)
4. **Observe** for 60 seconds:
   - Position drift
   - Altitude stability
   - Wind compensation
5. **Test wind response**:
   - Let wind push copter
   - Observe LQR corrections
   - Check if returns to position
6. **Land**

**Metrics to record:**
- Position drift: _____ meters
- Altitude variation: _____ meters
- Max wind speed: _____ m/s
- Control smoothness: Excellent / Good / Poor

### 4.2 Manual Position Commands

1. **Takeoff** to 10m in Mode 99
2. **Test roll commands**:
   - Roll stick right → should move east
   - Release → should hold new position
   - Roll stick left → should move west
3. **Test pitch commands**:
   - Pitch stick forward → should move north
   - Pitch stick back → should move south
4. **Test yaw commands**:
   - Yaw stick → should rotate smoothly
5. **Test altitude commands**:
   - Throttle up → should climb
   - Throttle down → should descend
   - Center → should hold altitude
6. **Land**

**✅ Pass criteria:**
- Smooth position changes
- Accurate position hold
- Good altitude hold
- Responsive but not jerky

---

## Phase 5: Advanced Flight Tests

### 5.1 Waypoint Navigation (with Companion Computer)

If using companion computer:

1. **Connect** companion computer
2. **Start** companion interface script
3. **Takeoff** and switch to Mode 99
4. **Send ROUTE_SET** from companion
5. **Execute** waypoint mission
6. **Monitor** telemetry throughout
7. **Land** automatically or manually

### 5.2 Wind Resistance Test

1. **Takeoff** to 20m
2. **Switch to Mode 99**
3. **Fly** in moderate wind (5-10 m/s)
4. **Observe**:
   - Position hold accuracy
   - LQR wind compensation
   - Control smoothness
   - Battery usage
5. **Land**

### 5.3 Altitude Range Test

1. **Takeoff** to various altitudes:
   - 5m, 10m, 20m, 50m
2. **Test** hover at each altitude
3. **Check** LQR performance at different heights
4. **Land**

---

## Phase 6: Long Duration Test

### 6.1 Extended Flight

1. **Fully charged battery**
2. **Takeoff** to 10m
3. **Switch to Mode 99**
4. **Hover** for 5-10 minutes
5. **Monitor**:
   - Battery percentage
   - LQR performance over time
   - Position drift accumulation
   - Control consistency
6. **Land** at 20% battery

**Metrics:**
- Flight time: _____ minutes
- Battery used: _____ %
- Average position error: _____ m
- LQR rate stability: _____ Hz

---

## Data Logging

### Essential Logs to Collect

**Flight Controller Logs (.bin files):**
- Located on SD card: `APM/LOGS/`
- Contains all telemetry at high rate
- Use Mission Planner to analyze

**Ground Control Telemetry (.tlog files):**
- Recorded by Mission Planner / MAVProxy
- Lower rate but includes GCS timestamps
- Good for timeline analysis

**Companion Computer Logs (if used):**
- Position commands sent
- State received
- Mission execution timeline

### Log Analysis

**Using Mission Planner:**
1. Open log file
2. Create graphs for:
   - Altitude vs Time
   - Position (North/East) vs Time
   - LQR_Thrust vs Time
   - LQR moments vs Time
   - Attitude (Roll/Pitch/Yaw) vs Time
3. Check for:
   - Oscillations
   - Step responses
   - Disturbance rejection

**Key Metrics:**
```
Position RMS error: < 0.5 m (good), < 1.0 m (acceptable)
Altitude RMS error: < 0.3 m (good), < 0.5 m (acceptable)
Attitude RMS error: < 5° (good), < 10° (acceptable)
Control rate: 100 Hz (consistent)
```

---

## Troubleshooting

### Problem: Won't Switch to Mode 99

**Checks:**
- [ ] Mode 99 assigned to flight mode channel
- [ ] GPS lock achieved (10+ satellites)
- [ ] EKF status good
- [ ] `sysid_params.txt` on SD card
- [ ] Firmware flashed correctly

**Solution:**
```
param set FLTMODE4 99
param fetch FLTMODE4
Reboot flight controller
```

---

### Problem: Oscillations in Mode 99

**Symptoms:** Rapid back-and-forth motions

**Causes:**
- Q matrix values too high
- R matrix values too low
- Poor PID tuning on inner loops
- Mechanical vibrations

**Solutions:**
1. Check vibration levels (< 30 m/s²)
2. Reduce Q values by 50%
3. Increase R values by 2×
4. Balance props, check frame rigidity

---

### Problem: Poor Position Hold

**Symptoms:** Drifts 2+ meters

**Causes:**
- Low Q position/velocity values
- Poor GPS accuracy
- Compass calibration issues
- Wind too strong

**Solutions:**
1. Increase Q_pos and Q_vel by 2×
2. Check GPS HDOP (< 1.5)
3. Recalibrate compass
4. Test in calmer conditions

---

### Problem: Altitude Drops

**Symptoms:** Loses altitude during maneuvers

**Causes:**
- Q_pos_d or Q_vel_d too low
- Hover throttle wrong
- Thrust calculation error

**Solutions:**
1. Increase Q_pos_d and Q_vel_d
2. Check `THROTTLE_HOVER` in sysid_params.txt
3. Verify mass in sysid_params.txt

---

## Safety Procedures

### Emergency Procedures

**During Flight Emergency:**
1. **Immediate**: Switch to Stabilize mode
2. **If uncontrolled**: Disarm (throttle down + yaw left)
3. **If still dangerous**: Use kill switch
4. **After landing**: Power off, inspect damage

**Lost Control Link:**
1. Flight controller enters RTL automatically (if configured)
2. Or continues current mode for 3 seconds then RTL
3. Always configure RTL home position

**Battery Low:**
1. Automatic warning at 20%
2. Automatic warning at 10%
3. Land immediately below 15%

### Pre-Flight Briefing Template

Before each flight, brief all personnel:

```
- Mission: Test Mode 99 hover
- Location: [GPS coordinates]
- Duration: 5 minutes
- Pilot: [Name]
- Safety officer: [Name]
- Emergency plan:
  * Lost link → RTL
  * Loss of control → Switch to Stabilize
  * Crash imminent → Disarm
- Abort triggers:
  * Oscillations
  * Unusual behavior
  * Equipment malfunction
  * Weather change
```

---

## Flight Test Log Template

```
Date: _________
Location: _________
Pilot: _________

Weather:
- Wind: _____ m/s
- Temperature: _____ °C
- Visibility: Good / Moderate / Poor

Copter:
- Total mass: _____ kg
- Battery: _____ mAh, _____ %
- Firmware: ArduCopter Mode 99 v_____

Test: _________

Pre-Flight Checks:
- [ ] GPS: _____ satellites, HDOP _____
- [ ] EKF: Healthy / Warning / Error
- [ ] Battery: _____ V
- [ ] Props: Secure, balanced
- [ ] Mode 99: Available in flight mode list

Flight Results:
- Flight time: _____ min
- Max altitude: _____ m
- Mode 99 active: _____ min
- Position RMS error: _____ m
- Altitude RMS error: _____ m
- Control rating: Excellent / Good / Fair / Poor

Issues:
- ____________________________________
- ____________________________________

Notes:
- ____________________________________
- ____________________________________

Recommendation:
- [ ] Pass - Continue testing
- [ ] Conditional pass - Tune and retest
- [ ] Fail - Investigate and fix
```

---

## Summary

**Testing Phases:**
1. ✅ Bench (Props OFF)
2. ✅ Ground (Props ON, Tethered)
3. ✅ First Hover
4. ✅ Position Control
5. ✅ Advanced Flight
6. ✅ Long Duration

**Safety First:**
- Always have emergency procedures ready
- Start conservative, increase complexity gradually
- Monitor telemetry constantly
- Abort at first sign of trouble

**Success Criteria:**
- Stable hover achieved
- Position hold < 1m error
- Altitude hold < 0.5m error
- LQR running at 100Hz consistently
- Smooth control, no oscillations

---

**Good luck with your testing! Fly safe! 🚁**
