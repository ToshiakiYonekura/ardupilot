// ArduCopter/mode_smartphoto99.h
// Smart Photo Mode (Mode 99) - LQI + Custom Quaternion EKF Control
//
// ============================================================================
// UNIT CONVENTIONS (STRICTLY ENFORCED)
// ============================================================================
// Position:       meters (NED frame: North, East, Down)
// Velocity:       m/s (NED frame)
// Attitude:       quaternion [q0=w, q1=x, q2=y, q3=z]
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
    const char *name() const override { return "SMARTPH99"; }
    const char *name4() const override { return "SP99"; }

private:
    // Safety monitoring flags
    struct SafetyState {
        bool battery_low;
        bool battery_critical;
        bool gps_healthy;
        bool ekf_healthy;
        uint32_t last_companion_msg_ms;
    } safety_state;

    // Safety configuration parameters
    static constexpr uint32_t COMPANION_TIMEOUT_MS = 12000;  // 12s: covers drain(1s)+wait_for_mode(5s)+M99_REF wait(5s)
    static constexpr float BATTERY_LOW_PERCENT = 30.0f;
    static constexpr float BATTERY_CRITICAL_PERCENT = 20.0f;
    static constexpr float MAX_WIND_SPEED_MS = 15.0f;

    // System identification data loaded from file
    struct SysidData {
        float mass;
        float Ixx, Iyy, Izz;
        float motor_kv;
        float max_thrust_per_motor;
        float arm_length;
        float moment_coefficient;
        float roll_rate_gain;
        float pitch_rate_gain;
        float yaw_rate_gain;
        float throttle_hover;
        uint32_t sample_count;
        bool parameters_loaded;
    } sysid_data;

    // =========================================================================
    // Custom Quaternion EKF
    // State: [pos_n, pos_e, pos_d, vel_n, vel_e, vel_d, q0, q1, q2, q3]
    // =========================================================================
    struct QuatEKF {
        static constexpr int N = 10;
        float x[N];         // State: [pos(3), vel(3), quat(4)]
        float P[N][N];      // Covariance matrix (symmetric)
        float q_pos;        // Process noise: position
        float q_vel;        // Process noise: velocity
        float q_att;        // Process noise: attitude (quaternion)
        float r_gps_pos;    // Measurement noise: GPS position
        float r_gps_vel;    // Measurement noise: GPS velocity
        uint32_t last_gps_fix_ms;
        bool initialized;
        // Scratch matrices for ekf_predict() — kept here to avoid stack allocation
        float _F[N][N];
        float _Phi[N][N];
        float _PhiP[N][N];
        float _PhiT[N][N];
        float _PhiPPhiT[N][N];
    } quat_ekf;

    // =========================================================================
    // LQI Integral States
    // =========================================================================
    struct LQIState {
        float int_pos_n, int_pos_e, int_pos_d;    // Position integral [m·s]
        float int_vel_n, int_vel_e, int_vel_d;    // Velocity integral [(m/s)·s]
        static constexpr float MAX_POS_INT = 10.0f;
        static constexpr float MAX_VEL_INT = 5.0f;
    } lqi_state;

    // State vector (uses EKF quaternion, not Euler)
    struct StateVector {
        float pos_n, pos_e, pos_d;
        float vel_n, vel_e, vel_d;
        float q0, q1, q2, q3;                  // Quaternion [w,x,y,z]
        float roll_rate, pitch_rate, yaw_rate;  // From gyro directly
    };

    // LQI gain matrix: K[4][18]
    // State error: [pos(3), vel(3), att_err(3), rate(3), int_pos(3), int_vel(3)]
    struct LQRGains {
        float K[4][18];
        bool valid;
        bool use_lqr;
    } lqr_gains;

    StateVector reference_state;
    StateVector current_state;

    float target_altitude;
    float target_yaw;
    Vector3f target_position_ne;

    // Companion computer command
    struct CompanionCommand {
        Vector3f position_ned;
        Vector3f velocity_ned;
        float yaw;
        float yaw_rate;
        uint32_t timestamp_ms;
        bool valid;
    } companion_cmd;

    // Smoothing parameters
    struct SmoothingParams {
        bool use_companion_cmd;
    } smoothing;

    // Timing control
    uint32_t last_state_feedback_ms;
    uint32_t last_wind_send_ms;
    uint32_t state_feedback_counter;
    uint32_t wind_send_counter;
    uint32_t mode_entry_ms;          // timestamp of mode init — used to detect "no cmd yet"
    uint32_t last_ref_broadcast_ms;  // last time M99_REF_* was broadcast
    static constexpr uint32_t STATE_FEEDBACK_DT_MS = 10;
    static constexpr uint32_t WIND_SEND_DT_MS = 10;

    // =========================================================================
    // Custom EKF functions
    // =========================================================================
    void ekf_init();
    void ekf_predict(float dt);
    void ekf_update_gps();
    void ekf_get_rotation_matrix(float R[3][3]) const;
    void ekf_compute_F_jacobian(float F[10][10], const float accel_b[3], const float gyro[3]) const;
    void mat10_multiply(const float A[10][10], const float B[10][10], float C[10][10]) const;
    void mat10_add(const float A[10][10], const float B[10][10], float C[10][10]) const;
    void mat10_transpose(const float A[10][10], float At[10][10]) const;

    // =========================================================================
    // LQI control functions
    // =========================================================================
    bool load_identified_parameters();
    void apply_identified_parameters();
    void calculate_lqr_gains();
    void get_ekf_states();
    void update_integral_states(float dt);
    void get_error_state_18(float e[18]) const;
    void compute_lqi_control();
    void use_attitude_controller_fallback();

    // Motor mixing
    void mix_motors_from_lqr(float F_total, float M_roll, float M_pitch, float M_yaw, float motor_thrust[4]);

    // Safety monitoring
    bool check_failsafes();  // returns true if a failsafe was triggered (mode changed)
    bool check_battery_level();
    bool check_gps_ekf_health();
    bool check_companion_heartbeat();
};
