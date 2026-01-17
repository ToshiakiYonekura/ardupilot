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
    const char *name() const override { return "SMARTPH99"; }
    const char *name4() const override { return "SP99"; }

private:
    // Mission state machine - New simplified 6-state design
    enum class MissionPhase : uint8_t {
        OUT_OF_MODE99 = 0,      // Not in mode 99
        PLANNING = 1,           // Waiting for ROUTE_SET from companion
        INITIALIZING = 2,       // Arming and rising to 50m altitude
        EXECUTING = 3,          // Executing autonomous mission
        COMPLETED = 4,          // Mission completed (reserved for future use)
        IDLE = 5                // Mission ended, disarming
    };

    struct MissionState {
        MissionPhase current_phase;
        MissionPhase previous_phase;
        uint32_t phase_start_time_ms;

        // Phase-specific flags
        bool route_set_received;        // ROUTE_SET message received from companion
        uint8_t companion_state;        // Last received companion state
        uint32_t last_companion_msg_ms; // Last companion message timestamp

        // Safety monitoring
        bool battery_low;
        bool battery_critical;
        bool gps_healthy;
        bool ekf_healthy;

        // Timing
        uint32_t landing_detect_start_ms;

        // Takeoff/landing parameters
        float takeoff_start_alt_m;
        float takeoff_target_alt_m;
    } mission_state;

    // Mission configuration parameters
    static constexpr float TAKEOFF_ALTITUDE_M = 50.0f;       // Target altitude above start
    static constexpr float ALTITUDE_THRESHOLD_M = 49.0f;     // Altitude threshold for state transition
    static constexpr float TAKEOFF_CLIMB_RATE_MS = 2.5f;     // Climb rate m/s
    static constexpr float LANDING_DESCENT_RATE_MS = 1.0f;   // Descent rate m/s
    static constexpr float LANDING_FINAL_RATE_MS = 0.5f;     // Final descent rate below 5m
    static constexpr uint32_t LANDING_STABILITY_MS = 2000;   // 2 second stability check
    static constexpr uint32_t COMPANION_TIMEOUT_MS = 500;    // 500ms command timeout
    static constexpr float BATTERY_LOW_PERCENT = 30.0f;      // Low battery warning
    static constexpr float BATTERY_CRITICAL_PERCENT = 20.0f; // Force landing
    static constexpr float MAX_WIND_SPEED_MS = 15.0f;        // Max safe wind speed

    // Mission state machine timing
    uint32_t last_state_machine_ms;
    uint32_t last_state_telemetry_ms;
    static constexpr uint32_t STATE_MACHINE_DT_MS = 100;     // 10Hz state machine update
    static constexpr uint32_t STATE_TELEMETRY_DT_MS = 100;   // 10Hz state telemetry

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
        // State feedback gains (LQR-like) - legacy structure
        float K_pos[3];      // Position gains
        float K_vel[3];      // Velocity gains
        float K_att[3];      // Attitude gains
        float K_rate[3];     // Rate gains
        bool gains_valid;
    } control_gains;

    // LQR gain matrix for momentum-based state feedback
    // State vector: [pos_n, pos_e, pos_d, vel_n, vel_e, vel_d, roll, pitch, yaw, p, q, r]
    // Control vector: [F_thrust, M_roll, M_pitch, M_yaw]
    struct LQRGains {
        float K[4][12];     // Gain matrix (4x12): u = -K * (x - x_ref)
        bool valid;         // Whether LQR gains have been computed
        bool use_lqr;       // Flag to enable LQR control (vs legacy control)
    } lqr_gains;

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

    // LQR momentum-based control functions
    void calculate_lqr_gains();
    void compute_lqr_state_feedback_control();
    void get_state_vector_12(float state[12]) const;
    void get_reference_vector_12(float ref_state[12]) const;

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
    void send_state_telemetry();

    // Phase handler functions
    void handle_out_of_mode99_phase();
    void handle_planning_phase();
    void handle_initializing_phase();
    void handle_executing_phase();
    void handle_completed_phase();
    void handle_idle_phase();

    // Transition check functions
    bool check_altitude_threshold_reached();
    bool check_destination_reached();
    bool check_landing_complete();

    // Companion interface
    void receive_route_set_command();

    // Safety monitoring functions
    void update_safety_flags();
    void check_emergency_conditions();
    bool check_battery_level();
    bool check_gps_ekf_health();
};
