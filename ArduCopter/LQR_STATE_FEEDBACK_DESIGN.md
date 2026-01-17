# LQR State Feedback Control Based on Momentum Equations

## Overview

This document describes the LQR (Linear Quadratic Regulator) state feedback controller implemented for Mode 99 in ArduCopter. The controller is based on the momentum equations of the copter and provides optimal control through full-state feedback.

## System Model

### State-Space Representation

The controller uses a **linearized state-space model** around hover:

```
ẋ = A·x + B·u
```

**State vector** (12 states):
```
x = [pos_n, pos_e, pos_d, vel_n, vel_e, vel_d, roll, pitch, yaw, p, q, r]ᵀ
```

Where:
- `pos_n, pos_e, pos_d`: Position in NED frame [meters]
- `vel_n, vel_e, vel_d`: Velocity in NED frame [m/s]
- `roll, pitch, yaw`: Euler angles [radians]
- `p, q, r`: Angular rates (body frame) [rad/s]

**Control vector** (4 inputs):
```
u = [F_thrust, M_roll, M_pitch, M_yaw]ᵀ
```

Where:
- `F_thrust`: Total thrust force [Newtons]
- `M_roll, M_pitch, M_yaw`: Body moments [N·m]

### Momentum Equations (Linearized)

#### Translational Dynamics

From Newton's second law: **F = m·a**

Linearized around hover with small angle approximation:

```
v̇_n = g·θ                    (pitch causes north acceleration)
v̇_e = -g·φ                   (roll causes east acceleration)
v̇_d = (F_thrust - m·g)/m     (thrust causes vertical acceleration)
```

Where:
- `g = 9.80665 m/s²` (gravity)
- `m` = copter mass [kg]
- `φ, θ` = roll, pitch angles [rad]

#### Rotational Dynamics

From angular momentum equation: **M = I·ω̇ + ω × (I·ω)**

Linearized (small rates, gyroscopic effects negligible):

```
ṗ = M_roll / Ixx
q̇ = M_pitch / Iyy
ṙ = M_yaw / Izz
```

Where:
- `Ixx, Iyy, Izz` = moments of inertia [kg·m²]

#### Kinematic Relations

For small angles:
```
φ̇ = p
θ̇ = q
ψ̇ = r
```

And position kinematics:
```
ṗ_n = v_n
ṗ_e = v_e
ṗ_d = v_d
```

### State-Space Matrices

**A matrix (12×12)** - System dynamics:
```
     [0  0  0  1  0  0  0  0  0  0  0  0]   pos_n
     [0  0  0  0  1  0  0  0  0  0  0  0]   pos_e
     [0  0  0  0  0  1  0  0  0  0  0  0]   pos_d
A =  [0  0  0  0  0  0  0  g  0  0  0  0]   vel_n
     [0  0  0  0  0  0 -g  0  0  0  0  0]   vel_e
     [0  0  0  0  0  0  0  0  0  0  0  0]   vel_d
     [0  0  0  0  0  0  0  0  0  1  0  0]   roll
     [0  0  0  0  0  0  0  0  0  0  1  0]   pitch
     [0  0  0  0  0  0  0  0  0  0  0  1]   yaw
     [0  0  0  0  0  0  0  0  0  0  0  0]   p
     [0  0  0  0  0  0  0  0  0  0  0  0]   q
     [0  0  0  0  0  0  0  0  0  0  0  0]   r
```

**B matrix (12×4)** - Control influence:
```
     [0      0        0         0    ]
     [0      0        0         0    ]
     [0      0        0         0    ]
B =  [0      0        0         0    ]
     [0      0        0         0    ]
     [1/m    0        0         0    ]
     [0      0        0         0    ]
     [0      0        0         0    ]
     [0      0        0         0    ]
     [0      1/Ixx    0         0    ]
     [0      0        1/Iyy     0    ]
     [0      0        0         1/Izz]
```

## LQR Controller Design

### Objective

Design optimal state feedback gain **K** to minimize the cost function:

```
J = ∫₀^∞ (xᵀ·Q·x + uᵀ·R·u) dt
```

Where:
- **Q** (12×12): State cost matrix (penalizes state deviations)
- **R** (4×4): Control cost matrix (penalizes control effort)

### Control Law

```
u = u_hover - K·(x - x_ref)
```

Where:
- `u_hover = [m·g, 0, 0, 0]ᵀ` (hover thrust, zero moments)
- `K` (4×12): LQR gain matrix
- `x_ref`: Reference/desired state

### Weighting Matrices

**Q matrix** (diagonal, state penalties):
```
Q = diag([1.0,  1.0,  2.0,    // Position [m]
          2.0,  2.0,  3.0,    // Velocity [m/s]
          10.0, 10.0, 5.0,    // Attitude [rad]
          1.0,  1.0,  0.5])   // Rates [rad/s]
```

**R matrix** (diagonal, control effort):
```
R = diag([0.1,  // Thrust [N]
          1.0,  // Roll moment [N·m]
          1.0,  // Pitch moment [N·m]
          2.0]) // Yaw moment [N·m]
```

**Tuning philosophy**:
- Higher Q → tighter tracking, more aggressive
- Higher R → smoother control, less aggressive
- Altitude (pos_d) weighted higher than horizontal position
- Attitude (roll/pitch) heavily penalized to stay level
- Yaw less critical than roll/pitch

### Gain Calculation

Full LQR requires solving the **Algebraic Riccati Equation (ARE)**:

```
AᵀP + PA - PBR⁻¹BᵀP + Q = 0
K = R⁻¹BᵀP
```

**Implementation approach** (embedded-friendly):

Due to computational constraints, we use a **simplified gain calculation** based on √(Q/R) scaling:

```c++
// Example for roll moment channel:
K[1][1] = √(Q[1]/R[1]) * g * scaling_factor      // pos_e → M_roll
K[1][4] = √(Q[4]/R[1]) * g * scaling_factor      // vel_e → M_roll
K[1][6] = √(Q[6]/R[1]) * Ixx * scaling_factor    // roll → M_roll
K[1][9] = √(Q[9]/R[1]) * Ixx * scaling_factor    // p → M_roll
```

This provides LQR-like gains that approximate the optimal solution.

### Gain Matrix Structure

```
K (4×12):
        pos_n  pos_e  pos_d  vel_n  vel_e  vel_d  roll   pitch  yaw    p      q      r
Thrust  [0     0      K₀₂    0      0      K₀₅    0      0      0      0      0      0   ]
M_roll  [0     K₁₁    0      0      K₁₄    0      K₁₆    0      0      K₁₉    0      0   ]
M_pitch [K₂₀   0      0      K₂₃    0      0      0      K₂₇    0      0      K₂₁₀   0   ]
M_yaw   [0     0      0      0      0      0      0      0      K₃₈    0      0      K₃₁₁]
```

**Physical interpretation**:
- **Thrust channel**: Controls altitude (pos_d) and climb rate (vel_d)
- **Roll moment**: Controls lateral position (pos_e), velocity, and roll angle
- **Pitch moment**: Controls forward position (pos_n), velocity, and pitch angle
- **Yaw moment**: Controls heading (yaw) and yaw rate

## Implementation Details

### Control Flow (100Hz)

```
1. Get EKF states → x_current (12-vector)
2. Get reference states → x_ref (12-vector)
3. Compute error → e = x_current - x_ref
4. Apply LQR law → u = u_hover - K·e
5. Constrain outputs for safety
6. Convert to attitude commands
7. Send to ArduPilot attitude controller
```

### Conversion to Attitude Commands

The LQR outputs thrust and moments, but ArduPilot's attitude controller expects:
- Roll/pitch angles [rad]
- Yaw rate [rad/s]
- Throttle [0-1]

**Conversion**:

```c++
// Throttle (normalized)
throttle = F_thrust / (m·g)

// Attitude angles from moments (empirical scaling)
roll_cmd = M_roll / k_roll_att      // k_roll_att ≈ 100 N·m/rad
pitch_cmd = M_pitch / k_pitch_att   // k_pitch_att ≈ 100 N·m/rad

// Yaw rate from moment
yaw_rate_cmd = M_yaw / k_yaw_rate   // k_yaw_rate ≈ 20 N·m/(rad/s)
```

These scaling factors are empirically tuned based on the actual attitude controller gains.

### Safety Limits

**Thrust limits**:
```c++
F_thrust ∈ [0.3·F_hover, 1.7·F_hover]
```

**Moment limits**:
```c++
M_roll, M_pitch ∈ [-50, 50] N·m
M_yaw ∈ [-20, 20] N·m
```

**Attitude limits**:
```c++
roll, pitch ∈ [-max_lean_angle, max_lean_angle]  // Typically ±30°
yaw_rate ∈ [-90°/s, 90°/s]
```

## Files

### Implementation Files

1. **mode_smartphoto99_lqr.cpp** (optional reference)
   - Standalone LQR functions
   - Can be used for offline gain calculation

2. **mode_smartphoto99.h**
   - LQRGains structure definition
   - Function declarations

3. **mode_smartphoto99.cpp**
   - Main implementation
   - Functions:
     - `calculate_lqr_gains()` - Compute K matrix
     - `compute_lqr_state_feedback_control()` - Main control loop
     - `get_state_vector_12()` - Pack state into array
     - `get_reference_vector_12()` - Pack reference into array

### Key Functions

```c++
// Initialize and calculate gains
void calculate_lqr_gains() {
    // Uses system ID parameters (mass, inertia)
    // Computes 4×12 gain matrix K
}

// Main control function (runs @ 100Hz)
void compute_lqr_state_feedback_control() {
    // 1. Get states from EKF
    // 2. Compute u = u_hover - K·e
    // 3. Convert to attitude commands
    // 4. Send to attitude controller
}
```

## Usage

### Enabling LQR Control

In mode init:
```c++
lqr_gains.use_lqr = true;   // Enable LQR (default)
lqr_gains.use_lqr = false;  // Use legacy cascaded control
```

### System ID Parameters Required

The LQR controller needs:
- `mass` [kg]
- `Ixx, Iyy, Izz` [kg·m²]
- `throttle_hover` [0-1]

These should be identified using Mode 98 (System ID mode) and saved to `sysid_params.txt`.

### Telemetry

The following values are sent at 100Hz for monitoring:
```
LQR_Thrust      // Total thrust [N]
LQR_M_roll      // Roll moment [N·m]
LQR_M_pitch     // Pitch moment [N·m]
LQR_M_yaw       // Yaw moment [N·m]
LQR_RollCmd     // Roll command [deg]
LQR_PitchCmd    // Pitch command [deg]
LQR_Throttle    // Throttle output [0-1]
LQR_Rate        // Execution rate [Hz]
```

## Advantages Over Cascaded Control

1. **Optimal Control**: LQR provides mathematically optimal gains
2. **Full-State Feedback**: Uses all 12 states simultaneously
3. **Coupled Control**: Naturally handles cross-coupling between axes
4. **Momentum-Based**: Directly based on physics equations
5. **Tunable**: Q and R matrices provide intuitive tuning

## Future Enhancements

1. **Full ARE Solution**: Implement iterative Riccati solver for true optimal gains
2. **Adaptive LQR**: Update gains based on changing conditions
3. **Integral Action**: Add integral states for disturbance rejection
4. **Nonlinear Control**: Extend to nonlinear momentum equations
5. **Direct Motor Commands**: Bypass attitude controller for even tighter control

## References

- Anderson, B.D.O. and Moore, J.B., "Optimal Control: Linear Quadratic Methods"
- Bouabdallah, S., "Design and control of quadrotors with application to autonomous flying"
- ArduPilot documentation on attitude control
- Linear state-space control theory

## Authors

- Implemented for ArduCopter Mode 99
- Based on momentum equations and LQR optimal control theory
- Date: January 2026
