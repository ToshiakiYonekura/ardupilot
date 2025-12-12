// ArduCopter/mode_smartphoto99.h
// Smart Photo Mode (Mode 99) - Uses identified parameters from mode 98

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
    struct StateVector {
        // Position states (NED frame)
        float pos_n, pos_e, pos_d;      // meters
        float vel_n, vel_e, vel_d;      // m/s

        // Attitude states
        float roll, pitch, yaw;          // radians
        float roll_rate, pitch_rate, yaw_rate;  // rad/s
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

    // Helper functions
    bool load_identified_parameters();
    void apply_identified_parameters();
    void calculate_state_feedback_gains();
    void get_ekf_states();
    void compute_state_feedback_control();
    void apply_motor_commands(const Vector3f& moment_cmd, float thrust_cmd);
};
