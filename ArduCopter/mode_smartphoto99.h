// ArduCopter/mode_smartphoto99.h
// Smart Photo Mode (Mode 99) - EKF State Feedback Control
//
// ============================================================================
// UNIT CONVENTIONS (STRICTLY ENFORCED)
// ============================================================================
// Position:       meters (NED frame: North, East, Down)
// Velocity:       m/s (NED frame)
// Angles:         radians (roll, pitch, yaw)
// Angular Rates:  rad/s (p, q, r in body frame)
// Time:           seconds (dt), milliseconds (timestamps)
// ============================================================================
// Companion computer MUST use these exact units for all MAVLink messages
// ============================================================================

#pragma once

#include "mode.h"
#include <AP_Math/AP_Math.h>

class ModeSmartPhoto99 : public Mode {
public:
    ModeSmartPhoto99();

    // Required overrides
    Number mode_number() const override { return Number(99); }
    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; }
    bool is_autopilot() const override { return true; }

    // Companion computer interface (public for MAVLink access)
    // Units: pos_ned [meters], vel_ned [m/s], yaw_target [radians], yaw_rate_target [rad/s]
    void update_companion_command(const Vector3f& pos_ned, const Vector3f& vel_ned,
                                   float yaw_target, float yaw_rate_target);
    bool companion_command_valid() const;

    // Mission sequence interface
    void set_mission_ready();
    void command_landing();
    uint8_t get_mission_phase() const;

protected:
    const char *name() const override { return "SMARTPHOTO"; }
    const char *name4() const override { return "SPHT"; }

private:
    // Mission state machine
    enum class MissionPhase : uint8_t {
        INITIALIZATION = 0,     // Companion sets waypoints/destination
        READY_TO_ARM = 1,       // Pre-arm checks, waiting for arm
        ARMED_WAITING = 2,      // 10-second safety wait after arming
        TAKEOFF = 3,            // Vertical climb to target altitude
        AUTONOMOUS_FLIGHT = 4,  // Companion-controlled flight
        LANDING = 5,            // Controlled descent to destination
        LANDED = 6,             // On ground, waiting to disarm
        DISARMED = 7,           // Mission complete
        EMERGENCY_LAND = 8,     // Emergency landing mode
        HOVER = 9               // Failsafe hover (companion timeout)
    };

    struct MissionState {
        MissionPhase current_phase;
        MissionPhase previous_phase;
        uint32_t phase_start_time_ms;

        // Phase-specific flags
        bool mission_configured;
        bool companion_ready;

        // Safety monitoring
        bool companion_timeout;
        bool battery_low;
        bool battery_critical;
        bool gps_healthy;
        bool ekf_healthy;

        // Timing
        uint32_t armed_wait_start_ms;
        uint32_t landing_detect_start_ms;
        uint32_t landed_stable_start_ms;

        // Takeoff/landing parameters
        float takeoff_start_alt_m;
        float takeoff_target_alt_m;
        float landing_target_alt_m;
    } mission_state;

    // Mission configuration parameters
    static constexpr float TAKEOFF_ALTITUDE_M = 50.0f;       // Target altitude above start
    static constexpr float TAKEOFF_CLIMB_RATE_MS = 2.5f;     // Climb rate m/s
    static constexpr float LANDING_DESCENT_RATE_MS = 1.0f;   // Descent rate m/s
    static constexpr float LANDING_FINAL_RATE_MS = 0.5f;     // Final descent rate below 5m
    static constexpr uint32_t ARMED_WAIT_TIME_MS = 10000;    // 10 second safety wait
    static constexpr uint32_t LANDING_STABILITY_MS = 2000;   // 2 second stability check
    static constexpr uint32_t COMPANION_TIMEOUT_MS = 500;    // 500ms command timeout
    static constexpr uint32_t COMPANION_FAILSAFE_MS = 5000;  // 5s timeout before landing
    static constexpr float BATTERY_LOW_PERCENT = 30.0f;      // Low battery warning
    static constexpr float BATTERY_CRITICAL_PERCENT = 20.0f; // Force landing
    static constexpr float MAX_WIND_SPEED_MS = 15.0f;        // Max safe wind speed

    // Mission state machine timing
    uint32_t last_state_machine_ms;
    static constexpr uint32_t STATE_MACHINE_DT_MS = 100;     // 10Hz state machine update

    // System identification data loaded from file
    struct SysidData {
        // Copter physical model
        float mass;                    // kg
        float Ixx, Iyy, Izz;          // Inertia moments

        // Motor configuration
        float motor_kv;                // KV value
        float max_thrust_per_motor;    // Max thrust per motor

        // Identified control parameters
        float roll_rate_gain;
        float pitch_rate_gain;
        float yaw_rate_gain;
        float throttle_hover;

        // Data collection info
        uint32_t sample_count;
        bool parameters_loaded;
    } sysid_data;

    // Target altitude for smart photo mode (maintain current altitude)
    float target_alt_cm;

    // State feedback control data
    // ALL UNITS: meters, m/s, radians, rad/s
    struct StateVector {
        // Position states (NED frame)
        float pos_n, pos_e, pos_d;      // [meters] North, East, Down
        float vel_n, vel_e, vel_d;      // [m/s] North, East, Down velocity

        // Attitude states
        float roll, pitch, yaw;          // [radians] Euler angles
        float roll_rate, pitch_rate, yaw_rate;  // [rad/s] Angular rates
    };

    struct ControlGains {
        // State feedback gains (LQR-like)
        float K_pos[3];      // Position gains
        float K_vel[3];      // Velocity gains
        float K_att[3];      // Attitude gains
        float K_rate[3];     // Rate gains
        bool gains_valid;
    } control_gains;

    StateVector reference_state;
    StateVector current_state;

    // Target setpoints (from pilot input)
    Vector3f target_position_ne;  // NE position target (m)
    float target_altitude;         // Down position target (m)
    float target_yaw;              // Yaw target (rad)

    // Companion computer command structure
    // ALL UNITS: meters, m/s, radians, rad/s
    struct CompanionCommand {
        Vector3f position_ned;     // Target position in NED [meters]
        Vector3f velocity_ned;     // Target velocity in NED [m/s]
        float yaw;                 // Target yaw [radians], 0 = North
        float yaw_rate;            // Target yaw rate [rad/s]
        uint32_t timestamp_ms;     // Last update time [milliseconds]
        bool valid;                // Command validity flag
    } companion_cmd;

    // Smooth attitude target generation
    // ALL UNITS: radians, rad/s
    struct AttitudeTarget {
        float roll;                // Target roll [radians]
        float pitch;               // Target pitch [radians]
        float yaw;                 // Target yaw [radians]
        float roll_rate;           // Target roll rate [rad/s]
        float pitch_rate;          // Target pitch rate [rad/s]
        float yaw_rate;            // Target yaw rate [rad/s]

        // Previous values for rate calculation
        float roll_prev;           // [radians]
        float pitch_prev;          // [radians]
        float yaw_prev;            // [radians]
        uint32_t last_update_ms;   // [milliseconds]
    } attitude_target;

    // Smoothing parameters
    struct SmoothingParams {
        float max_tilt_rate;       // Maximum tilt rate (rad/s) for smoothing
        float max_yaw_rate;        // Maximum yaw rate (rad/s)
        float attitude_tc;         // Time constant for attitude smoothing (s)
        bool use_companion_cmd;    // Use companion computer commands vs pilot input
    } smoothing;

    // Timing control for 100Hz loops
    uint32_t last_state_feedback_ms;    // Last time state feedback control ran
    uint32_t last_wind_send_ms;         // Last time wind data was sent
    uint32_t state_feedback_counter;    // Counter for state feedback executions
    uint32_t wind_send_counter;         // Counter for wind data transmissions
    static constexpr uint32_t STATE_FEEDBACK_DT_MS = 10;  // 100Hz = 10ms period
    static constexpr uint32_t WIND_SEND_DT_MS = 10;       // 100Hz = 10ms period

    // Helper functions
    bool load_identified_parameters();
    void apply_identified_parameters();
    void calculate_state_feedback_gains();
    void get_ekf_states();
    void compute_state_feedback_control();
    void use_attitude_controller_fallback();
    void apply_motor_commands(const Vector3f& moment_cmd, float thrust_cmd);

    // Smooth attitude generation
    void calculate_desired_attitude_from_velocity(const Vector3f& vel_cmd,
                                                   float& roll_target, float& pitch_target);
    void smooth_attitude_targets(float roll_desired, float pitch_desired, float yaw_desired,
                                 float dt);
    void calculate_attitude_rates(float dt);

    // Mission state machine functions
    void update_mission_state_machine();
    void transition_to_phase(MissionPhase new_phase);
    const char* get_phase_name(MissionPhase phase) const;

    // Phase handler functions
    void handle_initialization_phase();
    void handle_ready_to_arm_phase();
    void handle_armed_waiting_phase();
    void handle_takeoff_phase();
    void handle_autonomous_flight_phase();
    void handle_landing_phase();
    void handle_landed_phase();
    void handle_disarmed_phase();
    void handle_emergency_land_phase();
    void handle_hover_phase();

    // Transition check functions
    bool check_ready_to_arm();
    bool check_takeoff_complete();
    bool check_landing_complete();
    bool check_landed_stable();

    // Safety monitoring functions
    void update_safety_flags();
    void check_emergency_conditions();
    bool check_battery_level();
    bool check_gps_ekf_health();
};
