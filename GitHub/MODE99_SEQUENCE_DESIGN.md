# Mode 99 Autonomous Mission Sequence Design

## Overview
This document describes the state machine design for Mode 99 (SMARTPHOTO99) to enable autonomous missions with companion computer control.

---

## Mission Sequence Phases

### Phase 1: INITIALIZATION
**Description:** Companion computer sets destination and waypoints
**Duration:** Variable (until companion signals ready)
**Control:** Companion computer

**Actions:**
- Companion computer connects via MAVLink
- Sets mission waypoints/destination via MAVLink messages
- Configures mission parameters
- Validates GPS lock and position estimates
- Signals mission ready to flight controller

**Entry Conditions:**
- Mode 99 entered
- Vehicle disarmed
- Position estimate valid (GPS lock)

**Exit Conditions:**
- Mission waypoints received and validated
- Companion sends mission ready command
- Transition to → READY_TO_ARM

---

### Phase 2: READY_TO_ARM
**Description:** Control returns to flight controller, waiting for arm command
**Duration:** Variable (until pilot/companion arms)
**Control:** Flight controller

**Actions:**
- Pre-arm checks executed
- Motors initialized
- EKF state feedback gains calculated
- System ready indicator sent to companion
- Waiting for ARM command

**Entry Conditions:**
- Mission configured in INITIALIZATION
- Pre-arm checks pass
- Vehicle disarmed

**Exit Conditions:**
- ARM command received (pilot or companion)
- Transition to → ARMED_WAITING

**Safety Checks:**
- GPS lock valid
- Battery voltage sufficient
- No failsafe conditions
- EKF healthy

---

### Phase 3: ARMED_WAITING
**Description:** 10-second safety wait period after arming
**Duration:** 10 seconds
**Control:** Flight controller

**Actions:**
- Start 10-second countdown timer
- Monitor all sensors and systems
- Maintain throttle at minimum (motors spinning)
- Send countdown status to companion (1 Hz)
- Abort to disarm if any issues detected

**Entry Conditions:**
- Successfully armed in READY_TO_ARM
- All systems healthy

**Exit Conditions:**
- 10 seconds elapsed without issues
- Transition to → TAKEOFF
- OR abort to EMERGENCY_LAND if problems detected

**Safety Checks (continuous):**
- EKF innovation checks
- Battery voltage monitoring
- RC failsafe check
- GCS heartbeat monitoring
- Position estimate validity

---

### Phase 4: TAKEOFF
**Description:** Vertical takeoff to 50m altitude
**Duration:** ~15-20 seconds (depends on climb rate)
**Control:** Flight controller (altitude), Companion (horizontal position hold)

**Actions:**
- Set target altitude: current + 50m
- Climb rate: 2.5 m/s (configurable)
- Hold horizontal position from companion or hover at current location
- Monitor altitude progress
- Send altitude status to companion (10 Hz)

**Entry Conditions:**
- 10-second wait completed
- All systems healthy
- Clear to takeoff

**Exit Conditions:**
- Altitude target reached (within 1m tolerance)
- Climb rate < 0.2 m/s (stabilized)
- Transition to → AUTONOMOUS_FLIGHT
- OR abort to EMERGENCY_LAND if problems detected

**Safety Checks:**
- Climb rate reasonable (not stalling)
- Position hold within limits (5m horizontal)
- Battery sufficient for mission
- EKF healthy

**Companion Interface:**
- Companion can provide horizontal position targets during climb
- Flight controller controls vertical velocity
- If no companion commands, hover at takeoff location

---

### Phase 5: AUTONOMOUS_FLIGHT
**Description:** Mission execution with full companion control
**Duration:** Variable (until mission completion or landing command)
**Control:** Companion computer (position/velocity targets)

**Actions:**
- Accept position/velocity/yaw commands from companion via MAVLink
- Execute state feedback control at 100 Hz
- Send 3D wind estimates to companion at 100 Hz
- Send vehicle state (pos, vel, attitude) at 10 Hz
- Monitor companion command timeout (500ms)
- Execute waypoint navigation commanded by companion
- Companion decides when mission is complete

**Entry Conditions:**
- Takeoff completed and stabilized at 50m
- Companion sending valid commands
- All systems healthy

**Exit Conditions:**
- Companion commands landing (specific MAVLink message)
- Companion command timeout (fallback to pilot control)
- Battery low threshold reached
- Transition to → LANDING or EMERGENCY_LAND

**Companion Command Interface (via MAVLink SET_POSITION_TARGET_LOCAL_NED):**
```
Units: ALL in NED frame
- Position targets: [meters] (North, East, Down)
- Velocity targets: [m/s] (North, East, Down)
- Yaw target: [radians] (0 = North)
- Yaw rate: [rad/s]
- Update rate: 10-100 Hz (minimum 2 Hz)
```

**Flight Controller Responsibilities:**
- Execute low-level control (state feedback at 100 Hz)
- Safety monitoring and failsafes
- Motor control and attitude stabilization
- EKF state estimation

**Companion Computer Responsibilities:**
- High-level path planning
- Waypoint navigation
- Obstacle avoidance (if implemented)
- Mission logic
- Decide when to land

**Safety Checks:**
- Companion command timeout: 500ms → fallback to hover
- Geofence limits (if configured)
- Battery monitoring: < 30% → force landing
- EKF health monitoring
- Wind speed limits: > 10 m/s → warning to companion

---

### Phase 6: LANDING
**Description:** Controlled descent to destination
**Duration:** ~20-30 seconds (depends on altitude and descent rate)
**Control:** Flight controller (descent), Companion (horizontal position)

**Actions:**
- Set target altitude: ground level (0m AGL)
- Descent rate: 1.0 m/s initially, 0.5 m/s below 5m AGL
- Companion provides final landing position target
- Monitor rangefinder for ground detection
- Detect landing (low climb rate + on ground)
- Reduce throttle to minimum on touchdown

**Entry Conditions:**
- Companion sends landing command
- OR battery critical
- OR emergency landing required
- Vehicle position at or near destination

**Exit Conditions:**
- Land complete detected (rangefinder/weight on wheels)
- Motors at minimum throttle for 2 seconds
- Transition to → LANDED

**Safety Checks:**
- Descent rate controlled
- Horizontal position within tolerance (companion-commanded)
- Rangefinder valid for final approach
- Prevent hard landing (monitor descent rate)

**Landing Detection Criteria:**
- Rangefinder < 0.3m
- Climb rate < 0.1 m/s for 1 second
- Throttle at minimum
- No significant horizontal movement

---

### Phase 7: LANDED
**Description:** On ground, motors at minimum, preparing to disarm
**Duration:** 2 seconds
**Control:** Flight controller

**Actions:**
- Motors at minimum throttle (still armed)
- Wait 2 seconds for stability confirmation
- Verify no movement
- Send landing complete message to companion

**Entry Conditions:**
- Landing complete detected
- Vehicle stationary on ground

**Exit Conditions:**
- 2 seconds elapsed
- Vehicle confirmed stationary
- Transition to → DISARMED

---

### Phase 8: DISARMED
**Description:** Mission complete, motors disarmed
**Duration:** N/A (final state)
**Control:** Flight controller

**Actions:**
- Disarm motors automatically
- Stop all control loops
- Send mission complete message to companion
- Log mission statistics
- Ready for mode exit or restart

**Entry Conditions:**
- LANDED phase completed
- Vehicle safe to disarm

**Exit Conditions:**
- User exits Mode 99
- OR new mission initiated (return to INITIALIZATION)

---

## State Machine Diagram

```
INITIALIZATION
    ↓ (waypoints set)
READY_TO_ARM
    ↓ (arm command)
ARMED_WAITING
    ↓ (10 sec elapsed)
TAKEOFF
    ↓ (altitude reached)
AUTONOMOUS_FLIGHT ←→ (companion control loop)
    ↓ (landing command)
LANDING
    ↓ (touchdown detected)
LANDED
    ↓ (2 sec stability)
DISARMED
```

**Emergency Paths:**
- Any phase → EMERGENCY_LAND (if critical failure)
- AUTONOMOUS_FLIGHT → HOVER (if companion timeout)
- Any armed phase → DISARMED (if pilot force disarm)

---

## MAVLink Communication Protocol

### Messages from Companion to Flight Controller

#### 1. Mission Configuration (INITIALIZATION phase)
**Message:** `MISSION_ITEM_INT` or custom message
**Purpose:** Set waypoints and destination
**Fields:**
- Waypoint positions (lat/lon/alt or NED)
- Waypoint count
- Mission parameters

#### 2. Mission Ready Signal
**Message:** `COMMAND_LONG` with custom command ID
**Purpose:** Signal mission configured, ready to arm
**Command ID:** `MAV_CMD_USER_1` (custom, value 31010)

#### 3. Position/Velocity Commands (AUTONOMOUS_FLIGHT phase)
**Message:** `SET_POSITION_TARGET_LOCAL_NED` (ID: 84)
**Rate:** 10-100 Hz
**Purpose:** Real-time control during autonomous flight
**Fields:**
```
time_boot_ms: timestamp
target_system: 1
target_component: 1
coordinate_frame: MAV_FRAME_LOCAL_NED (1)
type_mask: (bit mask for which fields to use)
  - Bit 0-2: Position X,Y,Z
  - Bit 3-5: Velocity X,Y,Z
  - Bit 6-8: Acceleration X,Y,Z (not used)
  - Bit 9: Force
  - Bit 10: Yaw
  - Bit 11: Yaw rate
x, y, z: [meters] NED position targets
vx, vy, vz: [m/s] NED velocity targets
afx, afy, afz: [m/s²] acceleration (unused)
yaw: [radians] 0 = North, clockwise positive
yaw_rate: [rad/s] yaw rate
```

**Example type_mask values:**
- `0b0000111111111000` (0x0FF8): Use position + velocity + yaw + yaw_rate
- `0b0000111111000111` (0x0FC7): Use velocity only + yaw_rate
- `0b0000110111111000` (0x0DF8): Use position + velocity + yaw_rate (no yaw)

#### 4. Landing Command
**Message:** `COMMAND_LONG` with `MAV_CMD_NAV_LAND`
**Purpose:** Initiate landing sequence
**Parameters:**
- param5: Target latitude (or 0 for current)
- param6: Target longitude (or 0 for current)
- param7: Target altitude (0 for ground)

#### 5. Abort/Emergency Commands
**Message:** `COMMAND_LONG` with `MAV_CMD_NAV_RETURN_TO_LAUNCH` or custom
**Purpose:** Emergency abort

### Messages from Flight Controller to Companion

#### 1. System Status
**Message:** `HEARTBEAT` (ID: 0)
**Rate:** 1 Hz
**Purpose:** Keep-alive, mode status
**Fields:**
- custom_mode: 99 (Mode 99)
- system_status: armed state, failsafes

#### 2. State Machine Phase
**Message:** `STATUSTEXT` or custom telemetry
**Rate:** On change, or 1 Hz
**Purpose:** Current phase of mission sequence
**Content:** "MODE99: PHASE_TAKEOFF" etc.

#### 3. Vehicle State (AUTONOMOUS_FLIGHT phase)
**Message:** `LOCAL_POSITION_NED` (ID: 32)
**Rate:** 10 Hz
**Purpose:** Vehicle state feedback
**Fields:**
```
time_boot_ms: timestamp
x, y, z: [meters] NED position
vx, vy, vz: [m/s] NED velocity
```

**Additional:** `ATTITUDE` (ID: 30) at 10 Hz
```
roll, pitch, yaw: [radians]
rollspeed, pitchspeed, yawspeed: [rad/s]
```

#### 4. Wind Estimate
**Message:** `NAMED_VALUE_FLOAT` (ID: 251) at 100 Hz
**Purpose:** 3D wind estimate for companion planning
**Names:**
- "WindSpd": [m/s] horizontal magnitude
- "WindDir": [radians] direction, 0 = North
- "WindN": [m/s] North component
- "WindE": [m/s] East component
- "WindD": [m/s] Down component

#### 5. Safety Warnings
**Message:** `STATUSTEXT` (ID: 253)
**Rate:** On event
**Purpose:** Warnings, errors, phase transitions
**Examples:**
- "MODE99: Takeoff complete, autonomous flight enabled"
- "MODE99: Battery low, forcing landing"
- "MODE99: Companion timeout, holding position"

#### 6. Mission Complete
**Message:** `COMMAND_ACK` in response to landing
**Purpose:** Confirm mission sequence complete

---

## Safety Features and Failsafes

### 1. Companion Command Timeout
**Timeout:** 500ms without new commands
**Action:**
- AUTONOMOUS_FLIGHT → Hover at last position
- Send warning to companion
- If timeout > 5 seconds → Initiate return to launch or landing

### 2. Battery Monitoring
**Thresholds:**
- 30% remaining → Warning, suggest landing
- 20% remaining → Force LANDING phase
- 15% remaining → Emergency land at current position

**Actions:**
- Send battery status at 1 Hz
- Override companion commands if critical

### 3. GPS/EKF Health
**Monitors:**
- EKF innovation thresholds
- GPS satellite count (minimum 10)
- HDOP < 1.5
- Position estimate variance

**Actions:**
- If degraded during AUTONOMOUS_FLIGHT → Hold position
- If lost → Emergency land
- Prevent arming if no GPS lock

### 4. Geofence (optional)
**Limits:**
- Maximum altitude: configurable (default 120m)
- Horizontal boundary: configurable cylinder or polygon
- Minimum altitude: 2m AGL (prevent ground collision)

**Actions:**
- Soft limit: slow vehicle at boundary
- Hard limit: force stop at boundary
- Violation → Force landing if severe

### 5. Wind Speed Monitoring
**Thresholds:**
- 10 m/s → Warning to companion
- 15 m/s → Suggest landing
- 20 m/s → Force landing (unsafe)

**Rationale:** High winds can exceed control authority

### 6. RC Failsafe (if RC enabled)
**Trigger:** RC signal lost
**Action:** Continue mission if companion valid, else land

### 7. GCS Failsafe
**Trigger:** No GCS heartbeat for 5 seconds
**Action:** Continue mission (companion is primary control)

### 8. Manual Pilot Override
**Trigger:** Pilot switches mode or sends disarm
**Action:** Immediate exit from mode 99, respect pilot command

### 9. Rangefinder Check (LANDING phase)
**Monitor:** Rangefinder must be valid below 10m AGL
**Action:** If invalid, use barometer + GPS, reduce descent rate

### 10. Abnormal Descent Detection
**Monitor:** Unexpected descent rate > 3 m/s
**Action:** Assume control failure, increase throttle, emergency land

---

## Implementation Data Structures

### State Machine Enum
```cpp
enum class MissionPhase : uint8_t {
    INITIALIZATION = 0,
    READY_TO_ARM = 1,
    ARMED_WAITING = 2,
    TAKEOFF = 3,
    AUTONOMOUS_FLIGHT = 4,
    LANDING = 5,
    LANDED = 6,
    DISARMED = 7,
    EMERGENCY_LAND = 8,
    HOVER = 9  // Failsafe state
};
```

### Mission State Structure
```cpp
struct MissionState {
    MissionPhase current_phase;
    MissionPhase previous_phase;
    uint32_t phase_start_time_ms;

    // Phase-specific data
    bool mission_configured;
    bool companion_ready;
    uint8_t waypoint_count;
    uint8_t current_waypoint;

    // Safety flags
    bool companion_timeout;
    bool battery_low;
    bool gps_healthy;
    bool ekf_healthy;

    // Timing
    uint32_t armed_wait_start_ms;
    uint32_t landing_detect_start_ms;

    // Takeoff/landing parameters
    float takeoff_start_alt_m;
    float takeoff_target_alt_m;  // Usually +50m from start
    float landing_target_alt_m;
};
```

### Configuration Parameters (to add to mode 99)
```cpp
// Mission sequence parameters
float takeoff_altitude_m;         // Default: 50.0m
float takeoff_climb_rate_ms;      // Default: 2.5 m/s
float landing_descent_rate_ms;    // Default: 1.0 m/s
float landing_final_rate_ms;      // Default: 0.5 m/s below 5m
uint32_t armed_wait_time_ms;      // Default: 10000 (10 seconds)
uint32_t landing_stability_ms;    // Default: 2000 (2 seconds)

// Safety parameters
uint32_t companion_timeout_ms;    // Default: 500ms
float battery_low_percent;        // Default: 30%
float battery_critical_percent;   // Default: 20%
float max_wind_speed_ms;          // Default: 15 m/s
```

---

## State Transition Functions

### Core Transition Logic
```cpp
void ModeSmartPhoto99::update_mission_phase() {
    switch (mission_state.current_phase) {
        case MissionPhase::INITIALIZATION:
            handle_initialization_phase();
            break;
        case MissionPhase::READY_TO_ARM:
            handle_ready_to_arm_phase();
            break;
        case MissionPhase::ARMED_WAITING:
            handle_armed_waiting_phase();
            break;
        case MissionPhase::TAKEOFF:
            handle_takeoff_phase();
            break;
        case MissionPhase::AUTONOMOUS_FLIGHT:
            handle_autonomous_flight_phase();
            break;
        case MissionPhase::LANDING:
            handle_landing_phase();
            break;
        case MissionPhase::LANDED:
            handle_landed_phase();
            break;
        case MissionPhase::DISARMED:
            handle_disarmed_phase();
            break;
        case MissionPhase::EMERGENCY_LAND:
            handle_emergency_land_phase();
            break;
        case MissionPhase::HOVER:
            handle_hover_phase();
            break;
    }

    // Always check for emergency conditions
    check_emergency_conditions();
}
```

### Transition Criteria Checks
```cpp
bool ModeSmartPhoto99::check_ready_to_arm() {
    return mission_state.mission_configured &&
           mission_state.companion_ready &&
           copter.position_ok() &&
           !copter.any_failsafe_triggered();
}

bool ModeSmartPhoto99::check_takeoff_complete() {
    float alt_error = fabsf(current_state.pos_d - mission_state.takeoff_target_alt_m);
    float climb_rate = fabsf(current_state.vel_d);
    return (alt_error < 1.0f) && (climb_rate < 0.2f);
}

bool ModeSmartPhoto99::check_landing_complete() {
    float rangefinder_alt = copter.rangefinder_state.alt_cm * 0.01f;
    bool on_ground = (rangefinder_alt < 0.3f) || copter.ap.land_complete;
    bool low_climb_rate = fabsf(current_state.vel_d) < 0.1f;

    uint32_t now_ms = AP_HAL::millis();
    if (on_ground && low_climb_rate) {
        if (mission_state.landing_detect_start_ms == 0) {
            mission_state.landing_detect_start_ms = now_ms;
        }
        return (now_ms - mission_state.landing_detect_start_ms) > 1000;
    } else {
        mission_state.landing_detect_start_ms = 0;
        return false;
    }
}
```

---

## Integration with Existing Code

### Changes to mode_smartphoto99.h
1. Add `MissionPhase` enum
2. Add `MissionState` structure
3. Add configuration parameters
4. Add phase handler function declarations
5. Add MAVLink command handler declarations

### Changes to mode_smartphoto99.cpp
1. Initialize mission state in constructor
2. Add `update_mission_phase()` call to `run()` function
3. Implement phase handler functions
4. Modify `init()` to start in INITIALIZATION phase
5. Add safety check functions

### Changes to GCS_MAVLink_Copter.cpp
1. Add handler for mission ready command (MAV_CMD_USER_1)
2. Add mission configuration message handlers
3. Enhance SET_POSITION_TARGET_LOCAL_NED handling for phase awareness
4. Add telemetry for phase status

---

## Testing Plan

### Phase-by-Phase Testing

#### Test 1: INITIALIZATION → READY_TO_ARM
- Enter Mode 99
- Send waypoints via MAVLink
- Send mission ready command
- Verify phase transition

#### Test 2: READY_TO_ARM → ARMED_WAITING → TAKEOFF
- Arm vehicle
- Verify 10-second wait
- Verify countdown telemetry
- Verify takeoff initiation

#### Test 3: TAKEOFF → AUTONOMOUS_FLIGHT
- Monitor altitude climb
- Verify 50m target reached
- Verify phase transition
- Check companion control active

#### Test 4: AUTONOMOUS_FLIGHT waypoint navigation
- Send position commands
- Verify tracking
- Monitor state feedback
- Check wind telemetry

#### Test 5: AUTONOMOUS_FLIGHT → LANDING → DISARMED
- Send landing command
- Monitor descent
- Verify landing detection
- Verify auto-disarm

### Failsafe Testing

#### Test 6: Companion Timeout in AUTONOMOUS_FLIGHT
- Stop sending commands
- Verify 500ms timeout triggers hover
- Verify warning message
- Resume commands, verify recovery

#### Test 7: Battery Low During Flight
- Simulate battery voltage drop
- Verify forced landing initiation
- Verify override of companion commands

#### Test 8: GPS Loss During Flight
- Simulate GPS loss
- Verify emergency landing
- Verify safe descent

#### Test 9: Emergency Abort
- Send abort command mid-flight
- Verify immediate landing response

### Integration Testing

#### Test 10: Full Mission Sequence
- Complete mission from entry to disarm
- Multiple waypoints
- Verify all phases
- Log timing and performance

---

## Performance Requirements

### Timing Requirements
- State feedback control: 100 Hz (10ms period) ✓ (existing)
- Wind telemetry: 100 Hz ✓ (existing)
- State machine update: 10 Hz (100ms period) - NEW
- Companion command update: 2-100 Hz (existing)
- Phase transition latency: < 100ms
- Emergency response time: < 50ms

### Control Performance
- Position hold accuracy: ± 1m horizontal
- Altitude hold accuracy: ± 0.5m
- Velocity tracking error: < 0.2 m/s
- Attitude tracking error: < 5 degrees

### Communication Requirements
- MAVLink bandwidth: ~10 KB/s
- Latency: < 50ms radio link
- Packet loss tolerance: 5%

---

## Future Enhancements

1. **Multi-waypoint missions:** Store waypoint list in flight controller
2. **Return to Launch:** Automatic RTL on critical failure
3. **Dynamic obstacle avoidance:** Real-time path replanning
4. **Formation flight:** Multiple drones in Mode 99
5. **Advanced landing:** Precision landing on moving platform
6. **Mission pause/resume:** Hover and resume capability
7. **Terrain following:** Use rangefinder for low-altitude flight
8. **Dynamic geofence:** Update boundaries during flight

---

## Summary

This design implements a complete autonomous mission sequence for Mode 99 with:
- **8 mission phases** with clear entry/exit conditions
- **Companion computer control** during autonomous flight
- **10-second safety wait** after arming
- **Automatic takeoff to 50m** with position hold
- **Real-time position/velocity control** from companion
- **Safe landing and auto-disarm** at destination
- **Comprehensive failsafes** for safety
- **MAVLink protocol** for all communication
- **100 Hz state feedback control** (existing)
- **100 Hz wind telemetry** (existing)

The design maintains the existing EKF state feedback control architecture while adding mission sequencing on top.
