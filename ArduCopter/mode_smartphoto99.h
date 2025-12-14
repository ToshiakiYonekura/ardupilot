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

protected:
    const char *name() const override { return "SMARTPHOTO"; }
    const char *name4() const override { return "SPHT"; }

private:
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
};
