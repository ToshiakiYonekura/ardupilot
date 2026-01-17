# LQR Controller Tuning Guide - Q/R Matrix Adjustment

## Overview

The LQR controller performance is determined by two weighting matrices:
- **Q matrix (12×12)** - State cost (penalizes deviations from reference)
- **R matrix (4×4)** - Control cost (penalizes control effort)

Adjusting these matrices trades off between **tracking performance** and **control smoothness**.

---

## Understanding Q and R Matrices

### Q Matrix - State Cost (What You Care About)

```
Q = diag([q_pos_n, q_pos_e, q_pos_d,      # Position errors
          q_vel_n, q_vel_e, q_vel_d,      # Velocity errors
          q_roll, q_pitch, q_yaw,          # Attitude errors
          q_p, q_q, q_r])                  # Rate errors
```

**Higher Q value** → Care MORE about that state → Tighter control
**Lower Q value** → Care LESS about that state → Looser control

### R Matrix - Control Cost (How Much You Move)

```
R = diag([r_thrust,                        # Thrust changes
          r_roll_moment,                    # Roll moment
          r_pitch_moment,                   # Pitch moment
          r_yaw_moment])                    # Yaw moment
```

**Higher R value** → Penalize MORE control effort → Smoother, gentler
**Lower R value** → Allow MORE control effort → More aggressive

---

## Default Configuration

**Current settings** (in `mode_smartphoto99.cpp` line ~851):

```cpp
// Q matrix - State penalties
float Q_diag[12] = {
    1.0f,   // pos_n (m) - North position error
    1.0f,   // pos_e (m) - East position error
    2.0f,   // pos_d (m) - Down position error (altitude)
    2.0f,   // vel_n (m/s) - North velocity error
    2.0f,   // vel_e (m/s) - East velocity error
    3.0f,   // vel_d (m/s) - Down velocity error
    10.0f,  // roll (rad) - Roll attitude error
    10.0f,  // pitch (rad) - Pitch attitude error
    5.0f,   // yaw (rad) - Yaw attitude error
    1.0f,   // p (rad/s) - Roll rate error
    1.0f,   // q (rad/s) - Pitch rate error
    0.5f    // r (rad/s) - Yaw rate error
};

// R matrix - Control effort penalties
float R_diag[4] = {
    0.1f,   // F_thrust - Thrust effort (cheap)
    1.0f,   // M_roll - Roll moment effort
    1.0f,   // M_pitch - Pitch moment effort
    2.0f    // M_yaw - Yaw moment effort (expensive)
};
```

---

## Tuning Philosophy

### The Q/R Ratio Principle

For a given state and control, the gain approximately scales with:
```
K ≈ sqrt(Q/R)
```

**Example:** If `Q_roll = 10.0` and `R_roll_moment = 1.0`, then:
```
K_roll ≈ sqrt(10/1) = 3.16
```

To make roll control **2× more aggressive**:
- **Option 1**: Double Q: `Q_roll = 20.0` → `K ≈ 4.47` (2× increase)
- **Option 2**: Halve R: `R_roll_moment = 0.5` → `K ≈ 4.47` (2× increase)

---

## Common Tuning Scenarios

### Scenario 1: Tighter Position Hold

**Problem**: Drone drifts too much in wind

**Solution**: Increase position and velocity Q values

```cpp
float Q_diag[12] = {
    2.0f,   // pos_n ↑ (was 1.0)
    2.0f,   // pos_e ↑ (was 1.0)
    2.0f,   // pos_d (same)
    4.0f,   // vel_n ↑ (was 2.0)
    4.0f,   // vel_e ↑ (was 2.0)
    3.0f,   // vel_d (same)
    ...
};
```

**Effect**: More aggressive corrections for position errors

---

### Scenario 2: Smoother Flight

**Problem**: Flight is too jerky, oscillations

**Solution**: Increase R values (penalize control effort)

```cpp
float R_diag[4] = {
    0.2f,   // F_thrust ↑ (was 0.1)
    2.0f,   // M_roll ↑ (was 1.0)
    2.0f,   // M_pitch ↑ (was 1.0)
    3.0f    // M_yaw ↑ (was 2.0)
};
```

**Effect**: Gentler control inputs, smoother motion

---

### Scenario 3: Better Altitude Hold

**Problem**: Altitude fluctuates too much

**Solution**: Increase altitude Q values

```cpp
float Q_diag[12] = {
    1.0f,   // pos_n (same)
    1.0f,   // pos_e (same)
    4.0f,   // pos_d ↑ (was 2.0) - altitude
    2.0f,   // vel_n (same)
    2.0f,   // vel_e (same)
    6.0f,   // vel_d ↑ (was 3.0) - vertical velocity
    ...
};
```

**Effect**: Tighter altitude control

---

### Scenario 4: More Level Flight

**Problem**: Drone tilts too much

**Solution**: Increase attitude Q values

```cpp
float Q_diag[12] = {
    ...
    20.0f,  // roll ↑ (was 10.0)
    20.0f,  // pitch ↑ (was 10.0)
    5.0f,   // yaw (same)
    2.0f,   // p ↑ (was 1.0)
    2.0f,   // q ↑ (was 1.0)
    0.5f    // r (same)
};
```

**Effect**: Drone stays more level

---

### Scenario 5: Aggressive Racing Setup

**Problem**: Need fast, aggressive response

**Solution**: Increase Q, decrease R

```cpp
// More aggressive state tracking
float Q_diag[12] = {
    2.0f,   // pos_n ↑
    2.0f,   // pos_e ↑
    3.0f,   // pos_d ↑
    4.0f,   // vel_n ↑
    4.0f,   // vel_e ↑
    5.0f,   // vel_d ↑
    15.0f,  // roll ↑
    15.0f,  // pitch ↑
    8.0f,   // yaw ↑
    2.0f,   // p ↑
    2.0f,   // q ↑
    1.0f    // r ↑
};

// Allow more control effort
float R_diag[4] = {
    0.05f,  // F_thrust ↓
    0.5f,   // M_roll ↓
    0.5f,   // M_pitch ↓
    1.0f    // M_yaw ↓
};
```

**Effect**: Very responsive, aggressive control

---

### Scenario 6: Smooth Cinematic Flight

**Problem**: Need ultra-smooth video footage

**Solution**: Decrease Q, increase R

```cpp
// Less aggressive tracking
float Q_diag[12] = {
    0.5f,   // pos_n ↓
    0.5f,   // pos_e ↓
    1.0f,   // pos_d ↓
    1.0f,   // vel_n ↓
    1.0f,   // vel_e ↓
    1.5f,   // vel_d ↓
    5.0f,   // roll ↓
    5.0f,   // pitch ↓
    3.0f,   // yaw ↓
    0.5f,   // p ↓
    0.5f,   // q ↓
    0.25f   // r ↓
};

// Heavy penalty on control effort
float R_diag[4] = {
    0.5f,   // F_thrust ↑
    3.0f,   // M_roll ↑
    3.0f,   // M_pitch ↑
    5.0f    // M_yaw ↑
};
```

**Effect**: Very smooth, gentle movements

---

## Step-by-Step Tuning Process

### Step 1: Start with Defaults

Use the default values and observe:
- Position hold accuracy
- Altitude stability
- Attitude stability
- Control smoothness
- Response time

### Step 2: Identify Issues

Common problems:
- ❌ Drifting → Low Q_pos/Q_vel
- ❌ Oscillations → Low R or high Q_rate
- ❌ Sluggish → High R or low Q
- ❌ Altitude drops → Low Q_pos_d/Q_vel_d

### Step 3: Make Small Changes

**Rule of thumb**: Change values by 50% at a time

```cpp
// Too conservative? Increase by 1.5×
Q_diag[0] = 1.0f * 1.5f = 1.5f;

// Too aggressive? Decrease by 0.67×
Q_diag[0] = 1.0f * 0.67f = 0.67f;
```

### Step 4: Test and Iterate

After each change:
1. Rebuild: `./waf copter`
2. Flash to hardware
3. Test in safe environment
4. Record performance
5. Adjust as needed

### Step 5: Fine-Tune

Once close, make smaller adjustments (10-20%)

---

## How to Apply Changes

### Edit the Code

**File**: `ArduCopter/mode_smartphoto99.cpp`
**Function**: `calculate_lqr_gains()` (around line 851)

```cpp
void ModeSmartPhoto99::calculate_lqr_gains() {
    ...

    // YOUR CHANGES HERE
    float Q_diag[12] = {
        1.0f,   // pos_n - ADJUST THIS
        1.0f,   // pos_e - ADJUST THIS
        2.0f,   // pos_d - ADJUST THIS
        ...
    };

    float R_diag[4] = {
        0.1f,   // F_thrust - ADJUST THIS
        1.0f,   // M_roll - ADJUST THIS
        ...
    };

    ...
}
```

### Rebuild and Flash

```bash
cd ~/ardupilot

# Rebuild
./waf copter

# Flash to hardware
./waf copter --upload

# Or for specific board
./waf configure --board <your_board>
./waf copter --upload
```

---

## Advanced: Q/R Matrix Relationships

### Coupling Between States

Some states are naturally coupled:
- **Position ↔ Velocity**: Position errors cause velocity commands
- **Velocity ↔ Attitude**: Velocity errors cause attitude tilts
- **Attitude ↔ Rates**: Attitude errors cause rate commands

**Guideline**: Keep ratios consistent
```
Q_vel / Q_pos ≈ 2-4  (velocity more important than position)
Q_att / Q_rate ≈ 5-10 (attitude more important than rates)
```

### Control Authority

The ratio of Q to R determines **how much control authority** to use:

```
High Q/R → Use all available control
Low Q/R → Use minimal control
```

For safety, ensure:
- R values prevent saturation
- Q values don't cause oscillations

---

## Performance Metrics

After tuning, measure:

| Metric | Good | Needs Tuning |
|--------|------|--------------|
| Position RMS error | < 0.5 m | > 1.0 m |
| Altitude RMS error | < 0.3 m | > 0.5 m |
| Attitude RMS error | < 5° | > 10° |
| Control smoothness | No jerks | Visible oscillations |
| Response time | < 1 s | > 2 s |

---

## Troubleshooting

### Problem: Oscillations

**Symptoms**: Drone wobbles, high-frequency movements

**Causes**:
- Q too high relative to R
- Rate gains (Q_p, Q_q, Q_r) too high
- R too low

**Solutions**:
1. Increase R by 2×
2. Decrease Q_rate by 0.5×
3. Add damping: increase rate penalties

---

### Problem: Sluggish Response

**Symptoms**: Slow to reach commanded position, drifts

**Causes**:
- Q too low relative to R
- R too high
- Position/velocity gains too low

**Solutions**:
1. Increase Q_pos and Q_vel by 2×
2. Decrease R by 0.5×

---

### Problem: Altitude Drops

**Symptoms**: Loses altitude during maneuvers

**Causes**:
- Q_pos_d or Q_vel_d too low
- R_thrust too high
- Hover thrust estimate incorrect

**Solutions**:
1. Increase Q_pos_d and Q_vel_d by 2×
2. Check `sysid_params.txt` THROTTLE_HOVER value
3. Decrease R_thrust

---

### Problem: Tilts Excessively

**Symptoms**: Large attitude angles during position corrections

**Causes**:
- Q_roll, Q_pitch too low
- Position gains too high relative to attitude

**Solutions**:
1. Increase Q_roll and Q_pitch by 2×
2. Slightly decrease Q_pos and Q_vel

---

## Quick Reference Table

| Want More... | Increase | Decrease |
|--------------|----------|----------|
| Position hold accuracy | Q_pos, Q_vel | R_roll, R_pitch |
| Altitude stability | Q_pos_d, Q_vel_d | R_thrust |
| Level flight | Q_roll, Q_pitch, Q_p, Q_q | - |
| Smoothness | R (all) | Q (all) |
| Aggressiveness | Q (all) | R (all) |
| Response speed | Q_vel | R |

---

## Example Tuning Sessions

### Session 1: From Default to Smooth Cinema

**Initial**: Default values
**Issue**: Too aggressive for smooth video

**Changes**:
```cpp
// Iteration 1: Reduce aggressiveness
Q_diag = {0.5, 0.5, 1.0, 1.0, 1.0, 1.5, 5.0, 5.0, 3.0, 0.5, 0.5, 0.25};
R_diag = {0.3, 2.0, 2.0, 4.0};
```
**Result**: Smoother but still some jerks

**Changes**:
```cpp
// Iteration 2: Further reduce
Q_diag = {0.5, 0.5, 1.0, 1.0, 1.0, 1.5, 5.0, 5.0, 3.0, 0.5, 0.5, 0.25};
R_diag = {0.5, 3.0, 3.0, 5.0};  // Increased R
```
**Result**: ✓ Perfect for cinema

---

### Session 2: From Default to Tight Position Hold

**Initial**: Default values
**Issue**: Drifts 1-2m in wind

**Changes**:
```cpp
// Iteration 1: Increase position/velocity
Q_diag = {2.0, 2.0, 2.0, 4.0, 4.0, 3.0, 10.0, 10.0, 5.0, 1.0, 1.0, 0.5};
R_diag = {0.1, 1.0, 1.0, 2.0};  // Same
```
**Result**: Better but oscillates

**Changes**:
```cpp
// Iteration 2: Add damping
Q_diag = {2.0, 2.0, 2.0, 4.0, 4.0, 3.0, 10.0, 10.0, 5.0, 2.0, 2.0, 1.0};
R_diag = {0.1, 1.0, 1.0, 2.0};  // Same
```
**Result**: ✓ Tight hold, no oscillations

---

## Summary

**Key Principles**:
1. **Q controls tracking** - Higher Q = tighter control
2. **R controls smoothness** - Higher R = gentler control
3. **Q/R ratio determines aggressiveness**
4. **Start conservative, increase gradually**
5. **Test each change thoroughly**

**Safety First**:
- Always test in safe environment
- Start with low gains
- Increase gradually
- Monitor for oscillations
- Keep emergency controls ready

---

**Next Steps**: After tuning, document your final Q/R values for your specific application and hardware!
