# LQR Implementation - Detailed Explanation

## Overview

This document provides a detailed walkthrough of the LQR controller implementation, explaining every major component and how they work together.

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [State-Space Model](#state-space-model)
3. [Control Flow](#control-flow)
4. [Code Walkthrough](#code-walkthrough)
5. [Mathematical Details](#mathematical-details)
6. [Integration with ArduPilot](#integration-with-ardupilot)

---

## Architecture Overview

### High-Level Structure

```
┌─────────────────────────────────────────────────────────────┐
│                     Mode 99 LQR Controller                   │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌────────────┐  ┌──────────────┐  ┌─────────────────┐    │
│  │ Init Phase │→│ Calculate LQR │→│ Control Loop    │    │
│  │            │  │ Gains K(4×12) │  │ @ 100Hz         │    │
│  └────────────┘  └──────────────┘  └─────────────────┘    │
│                                              │               │
│                                              ↓               │
│  ┌────────────┐  ┌──────────────┐  ┌─────────────────┐    │
│  │ Get EKF    │→│ State Error   │→│ LQR Law         │    │
│  │ States (12)│  │ e = x - x_ref │  │ u = -K·e        │    │
│  └────────────┘  └──────────────┘  └─────────────────┘    │
│                                              │               │
│                                              ↓               │
│  ┌────────────┐  ┌──────────────┐  ┌─────────────────┐    │
│  │ Convert to │→│ ArduPilot     │→│ Motor Outputs   │    │
│  │ Attitude   │  │ Att Control   │  │                 │    │
│  └────────────┘  └──────────────┘  └─────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

### Key Components

| Component | File | Lines | Purpose |
|-----------|------|-------|---------|
| Mode Class | `mode_smartphoto99.h` | 21-243 | Class definition |
| Initialization | `mode_smartphoto99.cpp` | 104-174 | Mode init, load params |
| LQR Gains | `mode_smartphoto99.cpp` | 832-911 | Calculate K matrix |
| Control Loop | `mode_smartphoto99.cpp` | 946-1050 | Main LQR controller |
| EKF Interface | `mode_smartphoto99.cpp` | 458-504 | Get states from EKF |

---

## State-Space Model

### State Vector (12 states)

```cpp
struct StateVector {
    float pos_n, pos_e, pos_d;          // Position [m] NED frame
    float vel_n, vel_e, vel_d;          // Velocity [m/s] NED frame
    float roll, pitch, yaw;              // Attitude [rad]
    float roll_rate, pitch_rate, yaw_rate; // Rates [rad/s]
};
```

**Physical meaning:**
- **Position** `[pos_n, pos_e, pos_d]`: Where the copter is (meters from origin)
- **Velocity** `[vel_n, vel_e, vel_d]`: How fast it's moving (meters per second)
- **Attitude** `[roll, pitch, yaw]`: How it's tilted (radians)
- **Rates** `[p, q, r]`: How fast it's rotating (radians per second)

### Control Vector (4 inputs)

```cpp
float control_output[4];  // [F_thrust, M_roll, M_pitch, M_yaw]
```

**Physical meaning:**
- **F_thrust**: Total upward thrust force (Newtons)
- **M_roll**: Rolling moment (N·m) - makes copter roll left/right
- **M_pitch**: Pitching moment (N·m) - makes copter pitch forward/back
- **M_yaw**: Yawing moment (N·m) - makes copter rotate

### Linearized Dynamics

Around hover, the copter follows these approximate equations:

**Translation** (Newton's 2nd law: F = m·a):
```
v̇_n = g·θ                    (pitch angle causes north acceleration)
v̇_e = -g·φ                   (roll angle causes east acceleration)
v̇_d = (F_thrust - m·g) / m   (thrust difference causes vertical acceleration)
```

**Rotation** (Euler's equation: M = I·ω̇):
```
ṗ = M_roll / Ixx              (roll moment causes roll rate change)
q̇ = M_pitch / Iyy             (pitch moment causes pitch rate change)
ṙ = M_yaw / Izz               (yaw moment causes yaw rate change)
```

**Kinematics**:
```
ṗos = vel                     (position changes due to velocity)
φ̇ = p, θ̇ = q, ψ̇ = r          (attitude changes due to rates)
```

---

## Control Flow

### Step-by-Step Execution

#### 1. Initialization (`init()` - line 113)

```cpp
bool ModeSmartPhoto99::init(bool ignore_checks) {
    // Load system ID parameters (mass, inertia, hover throttle)
    load_identified_parameters();

    // Calculate LQR gains K(4×12)
    calculate_lqr_gains();

    // Get initial EKF states
    get_ekf_states();

    // Initialize reference to current state
    reference_state = current_state;
}
```

**What happens:**
1. Reads `sysid_params.txt` from SD card
2. Computes LQR gain matrix based on copter's mass and inertia
3. Gets current position, velocity, attitude from EKF
4. Sets reference (target) state to current state

#### 2. Main Loop (`run()` - line 162)

```cpp
void ModeSmartPhoto99::run() {
    // Get current time
    const uint32_t now_ms = AP_HAL::millis();

    // Always get current EKF states
    get_ekf_states();

    // Run state feedback @ 100Hz
    if (now_ms - last_state_feedback_ms >= 10) {
        compute_lqr_state_feedback_control();
    }
}
```

**What happens:**
1. Called at 400Hz (main loop rate)
2. Gets latest state estimates from EKF
3. Every 10ms (100Hz), runs LQR control calculation

#### 3. Get EKF States (`get_ekf_states()` - line 458)

```cpp
void ModeSmartPhoto99::get_ekf_states() {
    // Position from EKF (NED frame, meters)
    ahrs.get_relative_position_NED_origin(pos_ned);
    current_state.pos_n = pos_ned.x;
    current_state.pos_e = pos_ned.y;
    current_state.pos_d = pos_ned.z;

    // Velocity from EKF (NED frame, m/s)
    ahrs.get_velocity_NED(vel_ned);
    current_state.vel_n = vel_ned.x;
    current_state.vel_e = vel_ned.y;
    current_state.vel_d = vel_ned.z;

    // Attitude from AHRS (radians)
    current_state.roll = ahrs.get_roll();
    current_state.pitch = ahrs.get_pitch();
    current_state.yaw = ahrs.get_yaw();

    // Angular rates from gyro (rad/s)
    Vector3f gyro = ahrs.get_gyro();
    current_state.roll_rate = gyro.x;
    current_state.pitch_rate = gyro.y;
    current_state.yaw_rate = gyro.z;
}
```

**What happens:**
1. Queries EKF for position estimate (meters from origin)
2. Queries EKF for velocity estimate (meters per second)
3. Queries AHRS for attitude angles (roll, pitch, yaw in radians)
4. Queries gyro for angular rates (p, q, r in rad/s)

#### 4. LQR Control Calculation (`compute_lqr_state_feedback_control()` - line 946)

This is the heart of the controller. Let's break it down:

**Step 4a: Pack States into Vectors**

```cpp
// Get 12-element state vectors
float current_state_vec[12];
float reference_state_vec[12];
get_state_vector_12(current_state_vec);
get_reference_vector_12(reference_state_vec);
```

Converts state structures into arrays:
```
current_state_vec = [pos_n, pos_e, pos_d, vel_n, vel_e, vel_d,
                     roll, pitch, yaw, p, q, r]
```

**Step 4b: Compute State Error**

```cpp
// Compute state error: e = x - x_ref
float state_error[12];
for (int i = 0; i < 12; i++) {
    state_error[i] = current_state_vec[i] - reference_state_vec[i];
}

// Wrap yaw error to [-π, π]
state_error[8] = wrap_PI(state_error[8]);
```

Calculates how far current state is from desired state:
```
e_pos_n = current_pos_n - desired_pos_n
e_vel_n = current_vel_n - desired_vel_n
... (for all 12 states)
```

**Step 4c: Apply LQR Control Law**

```cpp
// u = u_hover - K·e
float control_output[4];

// Thrust control
control_output[0] = hover_thrust_N;
for (int j = 0; j < 12; j++) {
    control_output[0] -= lqr_gains.K[0][j] * state_error[j];
}

// Moment controls
for (int i = 1; i < 4; i++) {
    control_output[i] = 0.0f;
    for (int j = 0; j < 12; j++) {
        control_output[i] -= lqr_gains.K[i][j] * state_error[j];
    }
}
```

**Matrix multiplication:**
```
u_thrust = m·g - [K₀₀ K₀₁ ... K₀₁₁] · [e₀ e₁ ... e₁₁]ᵀ
u_M_roll = 0 - [K₁₀ K₁₁ ... K₁₁₁] · [e₀ e₁ ... e₁₁]ᵀ
u_M_pitch = 0 - [K₂₀ K₂₁ ... K₂₁₁] · [e₀ e₁ ... e₁₁]ᵀ
u_M_yaw = 0 - [K₃₀ K₃₁ ... K₃₁₁] · [e₀ e₁ ... e₁₁]ᵀ
```

**Example calculation for thrust:**
```
F_thrust = hover_thrust - (
    K[0][0] * e_pos_n +
    K[0][1] * e_pos_e +
    K[0][2] * e_pos_d +
    K[0][3] * e_vel_n +
    K[0][4] * e_vel_e +
    K[0][5] * e_vel_d +
    K[0][6] * e_roll +
    K[0][7] * e_pitch +
    K[0][8] * e_yaw +
    K[0][9] * e_p +
    K[0][10] * e_q +
    K[0][11] * e_r
)
```

**Step 4d: Apply Safety Limits**

```cpp
// Thrust limits
float min_thrust = hover_thrust_N * 0.3f;  // 30% min
float max_thrust = hover_thrust_N * 1.7f;  // 170% max
control_output[0] = constrain_float(control_output[0], min_thrust, max_thrust);

// Moment limits
control_output[1] = constrain_float(control_output[1], -50.0f, 50.0f);  // Roll
control_output[2] = constrain_float(control_output[2], -50.0f, 50.0f);  // Pitch
control_output[3] = constrain_float(control_output[3], -20.0f, 20.0f);  // Yaw
```

Prevents:
- Thrust too low (falling) or too high (damage)
- Moments too large (flipping)

**Step 4e: Convert to Attitude Commands**

```cpp
// Throttle (normalized 0-1)
float throttle_cmd = control_output[0] / (mass * gravity);

// Attitude from moments (empirical scaling)
float roll_cmd = control_output[1] / 100.0f;    // N·m → rad
float pitch_cmd = control_output[2] / 100.0f;   // N·m → rad
float yaw_rate_cmd = control_output[3] / 20.0f; // N·m → rad/s
```

**Why this conversion?**
- LQR computes **physical forces and moments** (Newtons, N·m)
- ArduPilot's attitude controller expects **angles and rates** (radians, rad/s)
- Empirical scaling factors (100, 20) convert between these
- Based on typical attitude controller gains

**Step 4f: Send to Attitude Controller**

```cpp
attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
    roll_cmd, pitch_cmd, yaw_rate_cmd
);

attitude_control->set_throttle_out(throttle_cmd, true, g.throttle_filt);
```

Hands off to ArduPilot's proven attitude controller for:
- Rate loop (converts attitude angles to rate commands)
- Motor mixing (converts rates to individual motor commands)
- Safety features (built into ArduPilot)

---

## Code Walkthrough

### Key Functions

#### `calculate_lqr_gains()` - Line 832

**Purpose:** Compute the 4×12 gain matrix K

```cpp
void ModeSmartPhoto99::calculate_lqr_gains() {
    // Define Q matrix (state cost)
    float Q_diag[12] = {
        1.0f,   // pos_n
        1.0f,   // pos_e
        2.0f,   // pos_d
        2.0f,   // vel_n
        2.0f,   // vel_e
        3.0f,   // vel_d
        10.0f,  // roll
        10.0f,  // pitch
        5.0f,   // yaw
        1.0f,   // p
        1.0f,   // q
        0.5f    // r
    };

    // Define R matrix (control cost)
    float R_diag[4] = {
        0.1f,   // F_thrust
        1.0f,   // M_roll
        1.0f,   // M_pitch
        2.0f    // M_yaw
    };

    // Compute gains using sqrt(Q/R) scaling
    lqr_gains.K[0][2] = sqrtf(Q_diag[2] / R_diag[0]) * mass * gravity * 0.5f;
    // ... (more gain calculations)
}
```

**How it works:**
1. Defines Q (how much we care about each state)
2. Defines R (how much we penalize control effort)
3. Computes gains using simplified formula: K ≈ √(Q/R) × scaling

**Full LQR** would solve Riccati equation, but this approximation works well for embedded systems.

#### `get_state_vector_12()` - Line 914

**Purpose:** Pack state structure into array for matrix operations

```cpp
void ModeSmartPhoto99::get_state_vector_12(float state[12]) const {
    state[0] = current_state.pos_n;
    state[1] = current_state.pos_e;
    state[2] = current_state.pos_d;
    state[3] = current_state.vel_n;
    state[4] = current_state.vel_e;
    state[5] = current_state.vel_d;
    state[6] = current_state.roll;
    state[7] = current_state.pitch;
    state[8] = current_state.yaw;
    state[9] = current_state.roll_rate;
    state[10] = current_state.pitch_rate;
    state[11] = current_state.yaw_rate;
}
```

**Why needed:**
- Internal state uses convenient structure
- Matrix math needs arrays
- This function bridges the two

---

## Mathematical Details

### LQR Theory (Simplified)

**Problem:** Design optimal gains K to minimize cost:
```
J = ∫₀^∞ (x^T·Q·x + u^T·R·u) dt
```

Where:
- `x^T·Q·x`: Cost of state deviations
- `u^T·R·u`: Cost of control effort

**Solution:** Solve Algebraic Riccati Equation (ARE):
```
A^T·P + P·A - P·B·R⁻¹·B^T·P + Q = 0
```

Then optimal gain is:
```
K = R⁻¹·B^T·P
```

**Simplified Implementation:**

For diagonal Q and R, and simple dynamics:
```
K_ij ≈ sqrt(Q_ii / R_jj) × scale_factor
```

This approximation:
- ✓ Avoids iterative matrix inversion
- ✓ Runs fast on embedded systems
- ✓ Provides good performance
- ✗ Not mathematically optimal (but close enough)

### Gain Matrix Structure

```
K (4×12):
        pos_n  pos_e  pos_d  vel_n  vel_e  vel_d  roll   pitch  yaw    p      q      r
Thrust  [0     0      ●      0      0      ●      0      0      0      0      0      0   ]
M_roll  [0     ●      0      0      ●      0      ●      0      0      ●      0      0   ]
M_pitch [●     0      0      ●      0      0      0      ●      0      0      ●      0   ]
M_yaw   [0     0      0      0      0      0      0      0      ●      0      0      ●   ]
```

**Physical interpretation:**
- **Thrust** controlled by: altitude (pos_d) and vertical velocity (vel_d)
- **Roll moment** controlled by: lateral position (pos_e), lateral velocity, roll angle, roll rate
- **Pitch moment** controlled by: forward position (pos_n), forward velocity, pitch angle, pitch rate
- **Yaw moment** controlled by: heading (yaw) and yaw rate

**Why these couplings?**
- Altitude → Thrust: Direct relationship
- Lateral position → Roll: Rolling moves laterally
- Forward position → Pitch: Pitching moves forward
- Heading → Yaw: Direct relationship

---

## Integration with ArduPilot

### How LQR Fits into ArduPilot

```
ArduPilot Flight Stack:
┌─────────────────────────────────────┐
│ Flight Modes (Stabilize, Loiter...) │
│         ↓                            │
│ ┌────────────────────────────────┐  │
│ │  Mode 99 LQR (This Code)       │  │
│ │  - Gets EKF states             │  │
│ │  - Computes optimal control    │  │
│ │  - Outputs attitude commands   │  │
│ └────────────┬───────────────────┘  │
│              ↓                       │
│ ┌────────────────────────────────┐  │
│ │ ArduPilot Attitude Controller  │  │
│ │ - Rate loops (P, I, D)        │  │
│ │ - Motor mixing                 │  │
│ │ - Safety limits                │  │
│ └────────────┬───────────────────┘  │
│              ↓                       │
│ ┌────────────────────────────────┐  │
│ │ Motors (ESCs, Props)           │  │
│ └────────────────────────────────┘  │
└─────────────────────────────────────┘
```

**Benefits of this architecture:**
- LQR does high-level optimal control
- ArduPilot handles low-level details (motor mixing, safety)
- Proven, tested attitude controller as safety net
- Easy to tune Q/R without touching PID loops

### Data Flow

**Inputs to LQR:**
- Position: From EKF (GPS + accelerometer fusion)
- Velocity: From EKF (GPS + accelerometer + gyro)
- Attitude: From AHRS (accelerometer + gyro + magnetometer)
- Rates: From gyro (direct measurement)

**Outputs from LQR:**
- Attitude angles (roll, pitch)
- Yaw rate
- Throttle

**Then ArduPilot:**
- Converts attitude angles to rate commands
- Runs PID loops on rates
- Mixes rates into individual motor commands
- Applies safety limits and failsafes

---

## Summary

### What Makes This LQR Special?

**1. Physics-Based:**
- Directly from momentum equations (F=ma, M=Iα)
- Not empirical tuning

**2. Optimal:**
- LQR theory provides mathematically optimal gains
- Balances performance vs control effort

**3. Full-State:**
- Uses all 12 states simultaneously
- Naturally handles cross-coupling

**4. Practical:**
- Simplified gain calculation for embedded systems
- Integrates with proven ArduPilot components
- Safety limits at every step

**5. Tunable:**
- Q/R matrices provide intuitive tuning
- Physical parameters (mass, inertia) from system ID

### Control Loop Summary

```
Every 10ms (100Hz):

1. Get states from EKF
   ↓
2. Compute error (current - reference)
   ↓
3. Apply LQR law: u = -K·e
   ↓
4. Limit for safety
   ↓
5. Convert to attitude commands
   ↓
6. Send to ArduPilot attitude controller
   ↓
7. ArduPilot handles motor mixing
   ↓
8. Motors respond
```

### Key Equations

**LQR Control Law:**
```
u = u_hover - K·(x - x_ref)
```

**State Vector:**
```
x = [pos_n, pos_e, pos_d, vel_n, vel_e, vel_d, roll, pitch, yaw, p, q, r]^T
```

**Control Vector:**
```
u = [F_thrust, M_roll, M_pitch, M_yaw]^T
```

**Gain Calculation:**
```
K_ij ≈ sqrt(Q_ii / R_jj) × physical_scaling
```

---

**This completes the detailed implementation explanation!**

For specific code questions, refer to:
- `mode_smartphoto99.h` - Class structure
- `mode_smartphoto99.cpp` - Implementation
- `LQR_STATE_FEEDBACK_DESIGN.md` - Mathematical derivation
