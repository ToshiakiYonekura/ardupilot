# Testing Mode 99 LQR State Feedback Controller

## Prerequisites

✅ ArduCopter built successfully
✅ System ID parameters file created: `sysid_params.txt`
✅ Terminal with display capability (for MAVProxy console and map)

## Quick Start

### Step 1: Launch SITL

Open your terminal and run:

```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --console --map
```

**What you'll see:**
- Build process (if needed)
- ArduCopter starting up
- MAVProxy console with `MAV>` prompt
- Map window (optional)
- Messages about GPS, EKF initialization

### Step 2: Wait for Ready State

Watch for these messages:
```
EKF2 IMU0 is using GPS
EKF2 IMU1 is using GPS
APM: EKF2 IMU0 origin set
APM: EKF2 IMU1 origin set
```

This means GPS lock is achieved and EKF is ready.

### Step 3: Check Pre-Flight Status

At the `MAV>` prompt, type:
```bash
status
```

Should show:
- GPS: Good (3D fix, 10+ satellites)
- EKF: Good
- Battery: OK

### Step 4: Arm the Vehicle

```bash
arm throttle
```

**Expected output:**
```
APM: Throttle armed
APM: GPS speed accuracy 0.3 below 1.0
```

If arming fails, check:
```bash
arm list     # Shows why arming is blocked
```

### Step 5: Switch to Mode 99 (LQR State Feedback)

```bash
mode 99
```

**🎯 CRITICAL: Watch for LQR Initialization Messages**

You should immediately see:

```
APM: SMARTPHOTO99: Using LQR momentum-based state feedback
APM: SMARTPHOTO99: LQR gains calculated
APM: Mass=2.00 kg, Hover=19.6 N
APM: SMARTPHOTO99: Entered PLANNING state
APM: SMARTPHOTO99: Waiting for ROUTE_SET from companion
```

If you see these messages → **LQR controller is working!** ✅

If you see "No sysid parameters found" → Check that `sysid_params.txt` exists in the ardupilot directory.

### Step 6: Take Off

```bash
rc 3 1650    # Increase throttle to 65%
```

The copter should:
- Start climbing smoothly
- Maintain level attitude
- Hold position

**Hover throttle:**
```bash
rc 3 1500    # Center throttle for hover
```

### Step 7: Monitor LQR Telemetry

In the MAVProxy console, you'll see telemetry streaming. Look for:

```
LQR_Thrust: ~19.6 N       # Should be close to mass × gravity
LQR_M_roll: ~0 N·m        # Small values for corrections
LQR_M_pitch: ~0 N·m       # Small values for corrections
LQR_M_yaw: ~0 N·m         # Small values for heading hold
LQR_RollCmd: ~0 deg       # Attitude commands
LQR_PitchCmd: ~0 deg
LQR_Throttle: ~0.5        # Normalized throttle
LQR_Rate: 100.0 Hz        # Execution rate (shows every 1 sec)
```

### Step 8: Test Position Control

Try these commands:

```bash
# Roll right (fly east)
rc 1 1700

# Return to center
rc 1 1500

# Pitch forward (fly north)
rc 2 1700

# Return to center
rc 2 1500

# Yaw right
rc 4 1700

# Return to center
rc 4 1500
```

**What to observe:**
- Smooth attitude changes
- LQR moments adjust to maintain commanded attitude
- Thrust adjusts to maintain altitude
- No oscillations or instability

### Step 9: Test Altitude Control

```bash
# Climb
rc 3 1600

# Descend
rc 3 1400

# Hover
rc 3 1500
```

**What to observe:**
- `LQR_Thrust` increases for climb, decreases for descent
- Smooth vertical motion
- Altitude hold when centered

### Step 10: Land and Disarm

```bash
# Descend slowly
rc 3 1300

# Wait for landing detection
# Vehicle will auto-disarm after landing

# Or manually disarm
disarm
```

### Step 11: Compare with Legacy Control

To test the difference:

```bash
# Arm and takeoff again
arm throttle
mode 99
rc 3 1650

# In mode_smartphoto99.cpp, the LQR is enabled by default
# To test legacy control, you'd need to modify code:
# lqr_gains.use_lqr = false;

# But for now, you can compare with other modes:
mode loiter    # Standard position hold
mode althold   # Altitude hold only
mode stabilize # Manual attitude control
```

## Advanced Testing

### View Detailed Telemetry

Create a second terminal and run:

```bash
cd ~/ardupilot/ArduCopter
tail -f mav.tlog | grep LQR
```

This will show all LQR telemetry messages.

### Log Analysis

After flight, analyze the log:

```bash
# Find the latest log
ls -lt logs/

# Use MAVProxy log analysis tools
mavlogdump.py --type NAMED_VALUE_FLOAT logs/[your-log].BIN | grep LQR
```

### Parameter Tuning

To adjust LQR tuning, edit `mode_smartphoto99.cpp`:

```cpp
// In calculate_lqr_gains() function (line ~851)

// Increase Q_diag values to make control more aggressive
float Q_diag[12] = {
    2.0f,   // pos_n (was 1.0)
    2.0f,   // pos_e (was 1.0)
    4.0f,   // pos_d (was 2.0)
    ...
};

// Increase R_diag values to make control smoother
float R_diag[4] = {
    0.2f,   // F_thrust (was 0.1)
    2.0f,   // M_roll (was 1.0)
    ...
};
```

Then rebuild:
```bash
cd ~/ardupilot
./waf copter
```

## Troubleshooting

### Problem: "No sysid parameters found"

**Solution:**
```bash
# Check file exists
ls -la ~/ardupilot/sysid_params.txt

# Check file contents
cat ~/ardupilot/sysid_params.txt

# If missing, it was created in wrong location
# The file should be in the ardupilot root directory
```

### Problem: Mode 99 not available

**Solution:**
```bash
# Check if mode is enabled
grep MODE_SMARTPHOTO_ENABLED ArduCopter/config.h

# Should show: #define MODE_SMARTPHOTO_ENABLED ENABLED
```

### Problem: Oscillations or instability

**Solution:**
1. Reduce Q matrix values (less aggressive)
2. Increase R matrix values (smoother control)
3. Check system ID parameters are realistic
4. Verify mass and inertia values

### Problem: Poor altitude hold

**Solution:**
1. Increase Q_diag[2] and Q_diag[5] (altitude and vertical velocity)
2. Check throttle hover value in sysid_params.txt
3. Verify thrust calculation: should be ~mass × gravity at hover

### Problem: Drifting position

**Solution:**
1. Increase Q_diag[0,1] and Q_diag[3,4] (horizontal position/velocity)
2. Check GPS quality (need 10+ satellites)
3. Verify EKF is healthy

## Expected Performance Metrics

| Metric | Expected Value | Notes |
|--------|---------------|-------|
| Control Rate | 100 Hz | LQR_Rate should show 100.0 |
| Hover Thrust | ~19.6 N | For 2kg copter (m×g) |
| Attitude Error | < 5° | Roll/pitch should be near zero |
| Position Hold | < 1 m drift | In calm conditions |
| Altitude Hold | ± 0.5 m | Vertical stability |
| Response Time | < 1 sec | Time to reach commanded attitude |

## Success Criteria

✅ LQR gains calculated successfully
✅ Controller runs at 100 Hz
✅ Stable hover achieved
✅ Smooth response to commands
✅ No oscillations
✅ Thrust approximately equals weight at hover
✅ Position hold in calm wind

## Data Collection

For analysis, collect:

1. **Console output** - Save all messages
2. **Telemetry log** - mav.tlog file
3. **DataFlash log** - .BIN file in logs/ directory
4. **Screenshots** - Console and map showing stable flight

## Next Steps After Successful Testing

1. ✅ Verify LQR implementation works
2. Compare performance with legacy controller
3. Tune Q and R matrices for your requirements
4. Test in wind conditions
5. Perform flight tests on actual hardware
6. Document flight performance

## Safety Notes

- Always test in SITL before hardware
- Start with conservative gains
- Monitor for oscillations
- Keep emergency controls ready (mode switch, disarm)
- Test altitude hold before position hold
- Verify all telemetry before aggressive maneuvers

---

**Ready to test!** Open your terminal and follow Step 1.
