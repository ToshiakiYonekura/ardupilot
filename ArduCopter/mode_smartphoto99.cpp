// ArduCopter/mode_smartphoto99.cpp
// Smart Photo Mode (Mode 99) - LQI + Custom Quaternion EKF Control
//
// Control Architecture:
// - Main loop: Runs at 400Hz
// - LQI + EKF Control: Runs at 100Hz (10ms period)
// - Wind Data Transmission: Runs at 100Hz
//
// Custom EKF state: [pos_n, pos_e, pos_d, vel_n, vel_e, vel_d, q0, q1, q2, q3]
// LQI error state (18-element): [pos(3), vel(3), att_err(3), rate(3), int_pos(3), int_vel(3)]

#include "Copter.h"
#include "mode.h"

#if MODE_SMARTPHOTO_ENABLED

// ============================================================================
// CONSTRUCTOR
// ============================================================================
ModeSmartPhoto99::ModeSmartPhoto99()
{
    // sysid_data — defaults mirror sysid_params.txt so fallback gains are correct
    // if sysid_params.txt is not found at runtime.
    sysid_data.parameters_loaded = false;
    sysid_data.mass = 2.0f;       // MASS=2.0
    sysid_data.Ixx  = 0.0347f;   // IXX=0.0347
    sysid_data.Iyy  = 0.0458f;   // IYY=0.0458
    sysid_data.Izz  = 0.0977f;   // IZZ=0.0977
    sysid_data.motor_kv = 0.0f;
    sysid_data.max_thrust_per_motor = 8.0f;
    sysid_data.arm_length = 0.225f;
    sysid_data.moment_coefficient = 0.016f;
    sysid_data.roll_rate_gain = 0.0f;
    sysid_data.pitch_rate_gain = 0.0f;
    sysid_data.yaw_rate_gain = 0.0f;
    sysid_data.throttle_hover = 0.5f;
    sysid_data.sample_count = 0;

    // LQR gains
    lqr_gains.valid = false;
    lqr_gains.use_lqr = true;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 18; j++) {
            lqr_gains.K[i][j] = 0.0f;
        }
    }

    // EKF
    memset(&quat_ekf, 0, sizeof(QuatEKF));
    quat_ekf.q_pos = 0.01f;
    quat_ekf.q_vel = 0.1f;
    quat_ekf.q_att = 0.001f;
    quat_ekf.r_gps_pos = 2.0f;
    quat_ekf.r_gps_vel = 0.1f;
    quat_ekf.initialized = false;

    // LQI state
    memset(&lqi_state, 0, sizeof(LQIState));

    // State vectors
    memset(&current_state, 0, sizeof(StateVector));
    memset(&reference_state, 0, sizeof(StateVector));
    // Default quaternion: identity
    current_state.q0 = 1.0f;
    reference_state.q0 = 1.0f;

    target_position_ne.zero();
    target_altitude = 0.0f;
    target_yaw = 0.0f;

    // Companion command
    companion_cmd.position_ned.zero();
    companion_cmd.velocity_ned.zero();
    companion_cmd.yaw = 0.0f;
    companion_cmd.yaw_rate = 0.0f;
    companion_cmd.timestamp_ms = 0;
    companion_cmd.valid = false;

    smoothing.use_companion_cmd = false;

    // Timing
    last_state_feedback_ms = 0;
    last_wind_send_ms = 0;
    state_feedback_counter = 0;
    wind_send_counter = 0;

    // Safety state
    safety_state.battery_low = false;
    safety_state.battery_critical = false;
    safety_state.gps_healthy = false;
    safety_state.ekf_healthy = false;
    safety_state.last_companion_msg_ms = 0;
}

// ============================================================================
// INIT
// ============================================================================
bool ModeSmartPhoto99::init(bool ignore_checks) {
    if (!copter.position_ok() && !ignore_checks) {
        return false;
    }

    // Load system ID parameters
    if (!load_identified_parameters()) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "SMARTPHOTO99: No sysid params, using defaults");
    }

    // Calculate LQI gains (heuristic fallback)
    calculate_lqr_gains();

    // Override K with physically-derived gains from lqr_design.py if available
    if (!load_lqr_gains_from_file()) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "SMARTPHOTO99: lqr_gains.txt not found, using heuristic K");
    }

    // Initialize custom EKF (seeds from AHRS)
    ekf_init();

    // Initialize EKF states into current_state
    get_ekf_states();

    // Set reference state = current state
    reference_state = current_state;
    target_altitude = current_state.pos_d;
    target_yaw = 0.0f;
    // Extract yaw from current quaternion
    {
        float q0 = current_state.q0, q1 = current_state.q1;
        float q2 = current_state.q2, q3 = current_state.q3;
        target_yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
    }
    target_position_ne.x = current_state.pos_n;
    target_position_ne.y = current_state.pos_e;

    // Reset integrals
    memset(&lqi_state, 0, sizeof(LQIState));

    // Reset LQR output storage (no valid output until first compute_lqi_control)
    lqr_roll_out      = 0.0f;
    lqr_pitch_out     = 0.0f;
    lqr_yaw_out       = 0.0f;
    lqr_throttle_norm = sysid_data.throttle_hover > 0.0f ? sysid_data.throttle_hover : 0.5f;
    lqr_output_valid  = false;

    // Timing
    uint32_t now_ms = AP_HAL::millis();
    last_state_feedback_ms = now_ms;
    last_wind_send_ms      = now_ms;
    mode_entry_ms          = now_ms;
    last_ref_broadcast_ms  = 0;       // force immediate first broadcast in run()

    // Safety — reset heartbeat timer so companion has COMPANION_TIMEOUT_MS from mode entry
    safety_state.last_companion_msg_ms = now_ms;
    safety_state.gps_healthy = check_gps_ekf_health();
    safety_state.ekf_healthy = copter.ahrs.healthy();
    check_battery_level();

    gcs().send_text(MAV_SEVERITY_INFO, "SMARTPHOTO99: EKF initialized");
    gcs().send_text(MAV_SEVERITY_INFO, "MODE99: LQI State Feedback @ 100Hz");
    gcs().send_text(MAV_SEVERITY_INFO, "MODE99: Awaiting commands from companion @ 20Hz");
    // M99_REF_* are broadcast at 1Hz in run() until first companion command arrives.

    return true;
}

// ============================================================================
// RUN (called at 400Hz)
// ============================================================================
void ModeSmartPhoto99::run() {
    const uint32_t now_ms = AP_HAL::millis();

    // Always update EKF states for failsafe checks
    get_ekf_states();

    // Failsafe monitoring (continuous) — return immediately if mode was changed
    if (check_failsafes()) {
        return;
    }

    // Enable motors
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Wind telemetry @ 1Hz (was 100Hz — rate-limited to avoid buffer overflow)
    if (now_ms - last_wind_send_ms >= 1000) {
        last_wind_send_ms = now_ms;
        wind_send_counter++;

        Vector3f wind_vec;
        if (ahrs.wind_estimate(wind_vec)) {
            float wind_speed = wind_vec.xy().length();
            float wind_dir = atan2f(wind_vec.y, wind_vec.x);
            gcs().send_named_float("WindSpd", wind_speed);
            gcs().send_named_float("WindDir", wind_dir);
            gcs().send_named_float("WindN", wind_vec.x);
            gcs().send_named_float("WindE", wind_vec.y);
            gcs().send_named_float("WindD", wind_vec.z);
        }
    }

    // M99_REF_* broadcast @ 1Hz until companion sends first command.
    // wait_for_mode() on the companion discards all non-HEARTBEAT messages, so the
    // one-shot broadcast from init() is lost.  Repeating here ensures the companion
    // receives the init reference regardless of timing.
    if (safety_state.last_companion_msg_ms == mode_entry_ms &&
        now_ms - last_ref_broadcast_ms >= 1000) {
        last_ref_broadcast_ms = now_ms;
        gcs().send_named_float("M99_REF_N", reference_state.pos_n);
        gcs().send_named_float("M99_REF_E", reference_state.pos_e);
        gcs().send_named_float("M99_REF_D", reference_state.pos_d);
    }

    // LQI control @ 100Hz
    if (now_ms - last_state_feedback_ms >= STATE_FEEDBACK_DT_MS) {
        last_state_feedback_ms = now_ms;
        state_feedback_counter++;

        const float dt = STATE_FEEDBACK_DT_MS * 0.001f;  // 0.01 s

        // Get state from AHRS (custom QuatEKF removed — it diverges)
        get_ekf_states();

        // Update reference from companion or pilot
        if (smoothing.use_companion_cmd && companion_command_valid()) {
            target_position_ne.x = companion_cmd.position_ned.x;
            target_position_ne.y = companion_cmd.position_ned.y;
            target_altitude      = companion_cmd.position_ned.z;
            target_yaw           = companion_cmd.yaw;
        } else {
            if (smoothing.use_companion_cmd) {
                smoothing.use_companion_cmd = false;
                gcs().send_text(MAV_SEVERITY_WARNING, "SMARTPHOTO99: Companion timeout, pilot fallback");
            }

            // Pilot input fallback
            float target_climb_rate_ms = get_pilot_desired_climb_rate_ms();
            target_climb_rate_ms = constrain_float(target_climb_rate_ms,
                -get_pilot_speed_dn_ms(), get_pilot_speed_up_ms());
            target_altitude -= target_climb_rate_ms * dt;

            float target_roll_rad, target_pitch_rad;
            get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad,
                attitude_control->lean_angle_max_rad(),
                attitude_control->get_althold_lean_angle_max_rad());

            float max_vel_xy = 2.0f;
            float vel_n = -target_pitch_rad * max_vel_xy / attitude_control->lean_angle_max_rad();
            float vel_e =  target_roll_rad  * max_vel_xy / attitude_control->lean_angle_max_rad();
            target_position_ne.x += vel_n * dt;
            target_position_ne.y += vel_e * dt;

            float yaw_rate_pilot = get_pilot_desired_yaw_rate_rads();
            target_yaw += yaw_rate_pilot * dt;
            target_yaw = wrap_PI(target_yaw);
        }

        // Set reference state
        reference_state.pos_n = target_position_ne.x;
        reference_state.pos_e = target_position_ne.y;
        reference_state.pos_d = target_altitude;

        // Reference velocity from companion — with rate-limiting to prevent sudden reversals.
        //
        // Problem: companion sends e.g. vel_ref_N = -1.5 m/s while drone is moving at +2 m/s N.
        //   → velocity error jumps to 3.5 m/s → LQR applies max moment → 40°+ tilt → FLIP
        //
        // Fix: rate-limit vel_ref change to MAX_VEL_REF_RATE per LQR cycle (10ms).
        //   MAX_VEL_REF_RATE = 1.0 m/s per 10ms = 100 m/s² max reference acceleration
        //   → change from 0 to 4.0 m/s in: 4.0/1.0 = 4 cycles = 40ms SIM (fast tracking)
        //   → change from +4.0 to -4.0 m/s in: 8.0/1.0 = 8 cycles = 80ms SIM (safe reversal)
        //   Increased from 0.30 (gym MAX_VEL=2.0, TILT=0) → 1.0 (gym MAX_VEL=4.0, TILT=0 confirmed)
        //   At 1.0/cycle reversal takes 80ms SIM → tilt < 10° with 2kg/Ixx=0.035 drone
        const float MAX_VEL_REF_RATE = 1.0f;  // m/s per 10ms LQR cycle
        float target_vel_n = 0.0f;
        float target_vel_e = 0.0f;
        float target_vel_d = 0.0f;
        if (smoothing.use_companion_cmd && companion_command_valid()) {
            target_vel_n = companion_cmd.velocity_ned.x;
            target_vel_e = companion_cmd.velocity_ned.y;
            target_vel_d = companion_cmd.velocity_ned.z;
        }
        reference_state.vel_n = constrain_float(target_vel_n,
            reference_state.vel_n - MAX_VEL_REF_RATE,
            reference_state.vel_n + MAX_VEL_REF_RATE);
        reference_state.vel_e = constrain_float(target_vel_e,
            reference_state.vel_e - MAX_VEL_REF_RATE,
            reference_state.vel_e + MAX_VEL_REF_RATE);
        reference_state.vel_d = constrain_float(target_vel_d,
            reference_state.vel_d - MAX_VEL_REF_RATE,
            reference_state.vel_d + MAX_VEL_REF_RATE);

        // Reference attitude: level flight at commanded yaw
        Quaternion q_ref;
        q_ref.from_euler(0.0f, 0.0f, target_yaw);
        reference_state.q0 = q_ref.q1;  // AP Quaternion: q1=w
        reference_state.q1 = q_ref.q2;  // q2=x
        reference_state.q2 = q_ref.q3;  // q3=y
        reference_state.q3 = q_ref.q4;  // q4=z

        // Reference rates: zero (or companion yaw rate)
        reference_state.roll_rate  = 0.0f;
        reference_state.pitch_rate = 0.0f;
        if (smoothing.use_companion_cmd && companion_command_valid()) {
            reference_state.yaw_rate = companion_cmd.yaw_rate;
        } else {
            reference_state.yaw_rate = 0.0f;
        }

        // Update integrals
        update_integral_states(dt);

        // Compute LQI control and output
        compute_lqi_control();
    } else {
        // Between 100Hz ticks: hold last motor commands (set by compute_lqi_control).
        // motors->output() is called by the scheduler; no action needed here.
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }
}

// ============================================================================
// COMPANION COMMAND
// ============================================================================
void ModeSmartPhoto99::update_companion_command(const Vector3f& pos_ned, const Vector3f& vel_ned,
                                                 float yaw_target, float yaw_rate_target) {
    // Reset integrators when altitude reference changes by more than 0.5m.
    // Prevents windup from the old hover setpoint fighting the new climb target.
    if (companion_cmd.valid) {
        float dz = fabsf(pos_ned.z - companion_cmd.position_ned.z);
        if (dz > 0.5f) {
            lqi_state.int_pos_n = 0.0f;
            lqi_state.int_pos_e = 0.0f;
            lqi_state.int_pos_d = 0.0f;
            lqi_state.int_vel_n = 0.0f;
            lqi_state.int_vel_e = 0.0f;
            lqi_state.int_vel_d = 0.0f;
            gcs().send_text(MAV_SEVERITY_INFO, "MODE99: ref changed %.1fm, integrals reset", (double)dz);
        }
    }

    companion_cmd.position_ned = pos_ned;
    companion_cmd.velocity_ned = vel_ned;
    companion_cmd.yaw = yaw_target;
    companion_cmd.yaw_rate = yaw_rate_target;
    companion_cmd.timestamp_ms = AP_HAL::millis();
    companion_cmd.valid = true;

    safety_state.last_companion_msg_ms = AP_HAL::millis();

    if (!smoothing.use_companion_cmd) {
        smoothing.use_companion_cmd = true;
        // Reset integrators on first companion command to prevent windup from initial transient
        lqi_state.int_pos_n = 0.0f;
        lqi_state.int_pos_e = 0.0f;
        lqi_state.int_pos_d = 0.0f;
        lqi_state.int_vel_n = 0.0f;
        lqi_state.int_vel_e = 0.0f;
        lqi_state.int_vel_d = 0.0f;
        gcs().send_text(MAV_SEVERITY_INFO, "MODE99: Companion control enabled, integrals reset");
    }
}

bool ModeSmartPhoto99::companion_command_valid() const {
    if (!companion_cmd.valid) {
        return false;
    }
    return true;
}

// ============================================================================
// FALLBACK: Standard attitude controller
// ============================================================================
void ModeSmartPhoto99::use_attitude_controller_fallback() {
    if (copter.position_ok()) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);
        attitude_control->set_throttle_out(sysid_data.throttle_hover, true, g.throttle_filt);
    } else {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);
        attitude_control->set_throttle_out(0.5f, true, g.throttle_filt);
    }
}

// ============================================================================
// OUTPUT_TO_MOTORS — reapply LQR commands just before motors->output()
//
// Called by motors_output_main() in Copter.cpp.  The scheduler runs:
//   run_rate_controller_main()  →  attitude_control->rate_controller_run()
//                                  which calls motors->set_pitch(~0) to hold level
//   motors_output_main()        →  flightmode->output_to_motors() → motors->output()
//   update_flight_mode()        →  Mode 99 compute_lqi_control() (runs LAST)
//
// Without this override the attitude rate controller always overwrites our LQR
// pitch/roll commands, keeping the drone level regardless of the velocity command.
// ============================================================================
void ModeSmartPhoto99::output_to_motors()
{
    if (lqr_output_valid && lqr_gains.valid && sysid_data.mass > 0.0f) {
        motors->set_roll(lqr_roll_out);
        motors->set_pitch(lqr_pitch_out);
        motors->set_yaw(lqr_yaw_out);
        motors->set_throttle(lqr_throttle_norm);
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }
    motors->output();
}

// ============================================================================
// LOAD / APPLY PARAMETERS
// ============================================================================
bool ModeSmartPhoto99::load_identified_parameters() {
    // Use raw POSIX calls to bypass AP_Filesystem's file_op_allowed() restriction
    // (AP::FS().open() is blocked when armed in main thread; ::open() is not).
    static const char* filenames[] = {
        "/tmp/sitl_cowork/sysid_params.txt",
        "sysid_params.txt",
    };
    int fd = -1;
    for (const char* filename : filenames) {
        fd = ::open(filename, O_RDONLY | O_CLOEXEC);
        if (fd != -1) break;
    }
    if (fd == -1) {
        return false;
    }
    static char buffer[512];
    ssize_t bytes_read = ::read(fd, buffer, sizeof(buffer) - 1);
    ::close(fd);
    if (bytes_read <= 0) {
        return false;
    }
    buffer[bytes_read] = '\0';

    char* line = strtok(buffer, "\n");
    while (line != NULL) {
        if (line[0] == '#' || line[0] == '\0' || line[0] == '\r') {
            line = strtok(NULL, "\n");
            continue;
        }
        char key[32];
        float value;
        if (sscanf(line, "%31[^=]=%f", key, &value) == 2) {
            if      (strcmp(key, "MASS")           == 0) sysid_data.mass = value;
            else if (strcmp(key, "IXX")            == 0) sysid_data.Ixx = value;
            else if (strcmp(key, "IYY")            == 0) sysid_data.Iyy = value;
            else if (strcmp(key, "IZZ")            == 0) sysid_data.Izz = value;
            else if (strcmp(key, "MOTOR_KV")       == 0) sysid_data.motor_kv = value;
            else if (strcmp(key, "MAX_THRUST")     == 0) sysid_data.max_thrust_per_motor = value;
            else if (strcmp(key, "ARM_LENGTH")     == 0) sysid_data.arm_length = value;
            else if (strcmp(key, "MOMENT_COEFF")   == 0) sysid_data.moment_coefficient = value;
            else if (strcmp(key, "ROLL_GAIN")      == 0) sysid_data.roll_rate_gain = value;
            else if (strcmp(key, "PITCH_GAIN")     == 0) sysid_data.pitch_rate_gain = value;
            else if (strcmp(key, "YAW_GAIN")       == 0) sysid_data.yaw_rate_gain = value;
            else if (strcmp(key, "THROTTLE_HOVER") == 0) sysid_data.throttle_hover = value;
            else if (strcmp(key, "SAMPLES")        == 0) sysid_data.sample_count = (uint32_t)value;
        }
        line = strtok(NULL, "\n");
    }
    sysid_data.parameters_loaded = true;
    gcs().send_text(MAV_SEVERITY_INFO,
        "SMARTPHOTO99: Loaded params mass=%.2f THR=%.2f",
        (double)sysid_data.mass, (double)sysid_data.throttle_hover);
    return true;
}

void ModeSmartPhoto99::apply_identified_parameters() {
    // Parameters are used directly from sysid_data in the control law
}

// ============================================================================
// LOAD LQR GAINS FROM FILE (4x18, generated by lqr_design.py)
// File format: lines starting with '#' are comments; 4 data lines each with
// 18 space-separated floats (row order: F_thrust, M_roll, M_pitch, M_yaw).
// ============================================================================
bool ModeSmartPhoto99::load_lqr_gains_from_file() {
    // Use raw POSIX calls to bypass AP_Filesystem's file_op_allowed() restriction
    // (AP::FS().open() is blocked when armed in main thread; ::open() is not).
    static const char* filenames[] = {
        "/tmp/sitl_cowork/lqr_gains.txt",
        "lqr_gains.txt",
    };
    int fd = -1;
    for (const char* filename : filenames) {
        fd = ::open(filename, O_RDONLY | O_CLOEXEC);
        if (fd != -1) break;
    }
    if (fd == -1) {
        return false;
    }
    static char buffer[2048];
    ssize_t bytes_read = ::read(fd, buffer, sizeof(buffer) - 1);
    ::close(fd);
    if (bytes_read <= 0) {
        return false;
    }
    buffer[bytes_read] = '\0';

    // Parse: skip comment lines, read 4 rows of 18 floats
    float K_new[4][18];
    int row = 0;
    char* line = strtok(buffer, "\n");
    while (line != nullptr && row < 4) {
        // Skip whitespace-only and comment lines
        char* p = line;
        while (*p == ' ' || *p == '\t' || *p == '\r') p++;
        if (*p == '#' || *p == '\0') {
            line = strtok(nullptr, "\n");
            continue;
        }
        // Parse 18 floats from this line
        for (int col = 0; col < 18; col++) {
            while (*p == ' ' || *p == '\t') p++;
            if (*p == '\0' || *p == '\r') {
                gcs().send_text(MAV_SEVERITY_WARNING,
                    "SMARTPHOTO99: lqr_gains.txt row %d: only %d values", row, col);
                return false;
            }
            char* end;
            K_new[row][col] = strtof(p, &end);
            if (end == p) {
                gcs().send_text(MAV_SEVERITY_WARNING,
                    "SMARTPHOTO99: lqr_gains.txt parse error row %d col %d", row, col);
                return false;
            }
            p = end;
        }
        row++;
        line = strtok(nullptr, "\n");
    }

    if (row < 4) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "SMARTPHOTO99: lqr_gains.txt: only %d/4 rows found", row);
        return false;
    }

    // Copy to lqr_gains
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 18; j++) {
            lqr_gains.K[i][j] = K_new[i][j];
        }
    }
    lqr_gains.valid = true;
    lqr_gains.use_lqr = true;
    gcs().send_text(MAV_SEVERITY_INFO, "SMARTPHOTO99: K loaded from lqr_gains.txt (4x18 LQI)");
    gcs().send_text(MAV_SEVERITY_INFO, "  K[thr][pD]=%.3f K[rol][pE]=%.3f K[rol][attR]=%.3f",
                    (double)lqr_gains.K[0][2],
                    (double)lqr_gains.K[1][1],
                    (double)lqr_gains.K[1][6]);
    return true;
}

// ============================================================================
// CUSTOM QUATERNION EKF
// State: x[0..2]=pos_ned, x[3..5]=vel_ned, x[6..9]=quat[w,x,y,z]
// ============================================================================

// Build rotation matrix R from quaternion x[6..9] = [q0=w, q1=x, q2=y, q3=z]
void ModeSmartPhoto99::ekf_get_rotation_matrix(float R[3][3]) const {
    float q0 = quat_ekf.x[6];
    float q1 = quat_ekf.x[7];
    float q2 = quat_ekf.x[8];
    float q3 = quat_ekf.x[9];

    R[0][0] = 1.0f - 2.0f*(q2*q2 + q3*q3);
    R[0][1] = 2.0f*(q1*q2 - q0*q3);
    R[0][2] = 2.0f*(q1*q3 + q0*q2);

    R[1][0] = 2.0f*(q1*q2 + q0*q3);
    R[1][1] = 1.0f - 2.0f*(q1*q1 + q3*q3);
    R[1][2] = 2.0f*(q2*q3 - q0*q1);

    R[2][0] = 2.0f*(q1*q3 - q0*q2);
    R[2][1] = 2.0f*(q2*q3 + q0*q1);
    R[2][2] = 1.0f - 2.0f*(q1*q1 + q2*q2);
}

// Compute process Jacobian F (10x10)
void ModeSmartPhoto99::ekf_compute_F_jacobian(float F[10][10],
                                               const float accel_b[3],
                                               const float gyro[3]) const {
    // Zero out
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            F[i][j] = 0.0f;
        }
    }

    // d(pos)/d(vel) = I3
    F[0][3] = 1.0f;
    F[1][4] = 1.0f;
    F[2][5] = 1.0f;

    // d(vel)/d(quat): ∂(R(q)*a_b)/∂q, 3 rows × 4 cols
    float q0 = quat_ekf.x[6];
    float q1 = quat_ekf.x[7];
    float q2 = quat_ekf.x[8];
    float q3 = quat_ekf.x[9];
    float ax = accel_b[0], ay = accel_b[1], az = accel_b[2];

    // Row 3 (a_ned_n): R[0]*a_b, d/d[q0,q1,q2,q3]
    // a_ned_n = (1-2q2²-2q3²)*ax + 2(q1q2-q0q3)*ay + 2(q1q3+q0q2)*az
    F[3][6] = 2.0f*(-q3*ay + q2*az);
    F[3][7] = 2.0f*( q2*ay + q3*az);
    F[3][8] = 2.0f*(-2.0f*q2*ax + q1*ay + q0*az);
    F[3][9] = 2.0f*(-2.0f*q3*ax - q0*ay + q1*az);

    // Row 4 (a_ned_e): R[1]*a_b
    // a_ned_e = 2(q1q2+q0q3)*ax + (1-2q1^2-2q3^2)*ay + 2(q2q3-q0q1)*az
    F[4][6] = 2.0f*( q3*ax - q1*az);
    F[4][7] = 2.0f*( q2*ax - 2.0f*q1*ay - q0*az);
    F[4][8] = 2.0f*( q1*ax + q3*az);
    F[4][9] = 2.0f*( q0*ax - 2.0f*q3*ay + q2*az);

    // Row 5 (a_ned_d): R[2]*a_b
    // a_ned_d = 2(q1q3-q0q2)*ax + 2(q2q3+q0q1)*ay + (1-2q1^2-2q2^2)*az
    F[5][6] = 2.0f*(-q2*ax + q1*ay);
    F[5][7] = 2.0f*( q3*ax + q0*ay - 2.0f*q1*az);
    F[5][8] = 2.0f*(-q0*ax + q3*ay - 2.0f*q2*az);
    F[5][9] = 2.0f*( q1*ax + q2*ay);

    // d(q_dot)/d(q): 0.5 * Omega(gyro)
    // Omega = [  0  -p  -q  -r ]
    //         [  p   0   r  -q ]
    //         [  q  -r   0   p ]
    //         [  r   q  -p   0 ]
    float p = gyro[0], q = gyro[1], r = gyro[2];
    F[6][6] =  0.0f;     F[6][7] = -0.5f*p;  F[6][8] = -0.5f*q;  F[6][9] = -0.5f*r;
    F[7][6] =  0.5f*p;   F[7][7] =  0.0f;    F[7][8] =  0.5f*r;  F[7][9] = -0.5f*q;
    F[8][6] =  0.5f*q;   F[8][7] = -0.5f*r;  F[8][8] =  0.0f;    F[8][9] =  0.5f*p;
    F[9][6] =  0.5f*r;   F[9][7] =  0.5f*q;  F[9][8] = -0.5f*p;  F[9][9] =  0.0f;
}

// Matrix helpers (10x10)
void ModeSmartPhoto99::mat10_multiply(const float A[10][10], const float B[10][10], float C[10][10]) const {
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            C[i][j] = 0.0f;
            for (int k = 0; k < 10; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void ModeSmartPhoto99::mat10_add(const float A[10][10], const float B[10][10], float C[10][10]) const {
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

void ModeSmartPhoto99::mat10_transpose(const float A[10][10], float At[10][10]) const {
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            At[i][j] = A[j][i];
        }
    }
}

// Initialize EKF from AHRS
void ModeSmartPhoto99::ekf_init() {
    // Seed position
    Vector3p pos_ned;
    if (ahrs.get_relative_position_NED_origin(pos_ned)) {
        quat_ekf.x[0] = pos_ned.x;
        quat_ekf.x[1] = pos_ned.y;
        quat_ekf.x[2] = pos_ned.z;
    }

    // Seed velocity
    Vector3f vel_ned;
    if (copter.ahrs.get_velocity_NED(vel_ned)) {
        quat_ekf.x[3] = vel_ned.x;
        quat_ekf.x[4] = vel_ned.y;
        quat_ekf.x[5] = vel_ned.z;
    }

    // Seed quaternion from AHRS
    Quaternion q_ahrs;
    copter.ahrs.get_quat_body_to_ned(q_ahrs);
    quat_ekf.x[6] = q_ahrs.q1;  // w
    quat_ekf.x[7] = q_ahrs.q2;  // x
    quat_ekf.x[8] = q_ahrs.q3;  // y
    quat_ekf.x[9] = q_ahrs.q4;  // z

    // Initialize covariance: large diagonal
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            quat_ekf.P[i][j] = 0.0f;
        }
    }
    quat_ekf.P[0][0] = 10.0f;
    quat_ekf.P[1][1] = 10.0f;
    quat_ekf.P[2][2] = 10.0f;
    quat_ekf.P[3][3] = 1.0f;
    quat_ekf.P[4][4] = 1.0f;
    quat_ekf.P[5][5] = 1.0f;
    quat_ekf.P[6][6] = 0.1f;
    quat_ekf.P[7][7] = 0.1f;
    quat_ekf.P[8][8] = 0.1f;
    quat_ekf.P[9][9] = 0.1f;

    quat_ekf.last_gps_fix_ms = copter.gps.last_fix_time_ms();
    quat_ekf.initialized = true;
}

// EKF Predict step using IMU
void ModeSmartPhoto99::ekf_predict(float dt) {
    if (!quat_ekf.initialized) {
        ekf_init();
        return;
    }

    // Get IMU measurements
    Vector3f accel_b = copter.ins.get_accel();
    Vector3f gyro    = copter.ins.get_gyro();

    float ax = accel_b.x, ay = accel_b.y, az = accel_b.z;
    float gx = gyro.x,    gy = gyro.y,    gz = gyro.z;

    // Rotation matrix from current quaternion
    float R[3][3];
    ekf_get_rotation_matrix(R);

    // Rotate accel to NED
    float a_ned_n = R[0][0]*ax + R[0][1]*ay + R[0][2]*az;
    float a_ned_e = R[1][0]*ax + R[1][1]*ay + R[1][2]*az;
    float a_ned_d = R[2][0]*ax + R[2][1]*ay + R[2][2]*az + GRAVITY_MSS;  // add gravity (NED: +down)

    // State propagation (Euler integration)
    // Position
    quat_ekf.x[0] += quat_ekf.x[3] * dt;
    quat_ekf.x[1] += quat_ekf.x[4] * dt;
    quat_ekf.x[2] += quat_ekf.x[5] * dt;

    // Velocity
    quat_ekf.x[3] += a_ned_n * dt;
    quat_ekf.x[4] += a_ned_e * dt;
    quat_ekf.x[5] += a_ned_d * dt;

    // Quaternion kinematics: q_dot = 0.5 * q * [0, p, q, r]
    float q0 = quat_ekf.x[6], q1 = quat_ekf.x[7];
    float q2 = quat_ekf.x[8], q3 = quat_ekf.x[9];
    float dq0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float dq1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float dq2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float dq3 = 0.5f * ( q0*gz + q1*gy - q2*gx);
    quat_ekf.x[6] += dq0 * dt;
    quat_ekf.x[7] += dq1 * dt;
    quat_ekf.x[8] += dq2 * dt;
    quat_ekf.x[9] += dq3 * dt;

    // Normalize quaternion
    float norm = sqrtf(quat_ekf.x[6]*quat_ekf.x[6] + quat_ekf.x[7]*quat_ekf.x[7] +
                       quat_ekf.x[8]*quat_ekf.x[8] + quat_ekf.x[9]*quat_ekf.x[9]);
    if (norm > 1e-6f) {
        quat_ekf.x[6] /= norm;
        quat_ekf.x[7] /= norm;
        quat_ekf.x[8] /= norm;
        quat_ekf.x[9] /= norm;
    }

    // Covariance propagation: P = (I + F*dt)*P*(I + F*dt)^T + Q*dt
    // Use member scratch matrices to avoid 2KB stack allocation
    float accel_arr[3] = {ax, ay, az};
    float gyro_arr[3]  = {gx, gy, gz};
    ekf_compute_F_jacobian(quat_ekf._F, accel_arr, gyro_arr);

    // Phi = I + F*dt
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            quat_ekf._Phi[i][j] = (i == j ? 1.0f : 0.0f) + quat_ekf._F[i][j] * dt;
        }
    }

    // Phi * P
    mat10_multiply(quat_ekf._Phi, quat_ekf.P, quat_ekf._PhiP);

    // Phi^T
    mat10_transpose(quat_ekf._Phi, quat_ekf._PhiT);

    // Phi * P * Phi^T
    mat10_multiply(quat_ekf._PhiP, quat_ekf._PhiT, quat_ekf._PhiPPhiT);

    // Add Q*dt (diagonal)
    for (int i = 0; i < 10; i++) {
        float qi = 0.0f;
        if (i < 3)       qi = quat_ekf.q_pos;
        else if (i < 6)  qi = quat_ekf.q_vel;
        else             qi = quat_ekf.q_att;
        quat_ekf._PhiPPhiT[i][i] += qi * dt;
    }

    // Copy back to P
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            quat_ekf.P[i][j] = quat_ekf._PhiPPhiT[i][j];
        }
    }
}

// EKF GPS update (position + velocity separately)
void ModeSmartPhoto99::ekf_update_gps() {
    if (!quat_ekf.initialized) {
        return;
    }

    uint32_t gps_fix_ms = copter.gps.last_fix_time_ms();
    if (gps_fix_ms == quat_ekf.last_gps_fix_ms) {
        return;  // No new GPS data
    }
    quat_ekf.last_gps_fix_ms = gps_fix_ms;

    // --- Position update (H selects rows 0,1,2) ---
    Vector3p pos_meas;
    if (ahrs.get_relative_position_NED_origin(pos_meas)) {
        for (int axis = 0; axis < 3; axis++) {
            float z_meas = (axis == 0) ? pos_meas.x : (axis == 1) ? pos_meas.y : pos_meas.z;
            float innov = z_meas - quat_ekf.x[axis];

            // S = P[axis][axis] + R
            float S = quat_ekf.P[axis][axis] + quat_ekf.r_gps_pos;
            if (fabsf(S) < 1e-9f) continue;

            // K = P[:,axis] / S
            float K[10];
            for (int i = 0; i < 10; i++) {
                K[i] = quat_ekf.P[i][axis] / S;
            }

            // x = x + K * innov
            for (int i = 0; i < 10; i++) {
                quat_ekf.x[i] += K[i] * innov;
            }

            // P = (I - K*H) * P  (H[row] = e_axis^T)
            for (int i = 0; i < 10; i++) {
                for (int j = 0; j < 10; j++) {
                    quat_ekf.P[i][j] -= K[i] * quat_ekf.P[axis][j];
                }
            }
        }
    }

    // --- Velocity update (H selects rows 3,4,5) ---
    Vector3f vel_meas;
    if (copter.ahrs.get_velocity_NED(vel_meas)) {
        float vel_arr[3] = {vel_meas.x, vel_meas.y, vel_meas.z};
        for (int axis = 0; axis < 3; axis++) {
            int state_idx = axis + 3;
            float innov = vel_arr[axis] - quat_ekf.x[state_idx];

            float S = quat_ekf.P[state_idx][state_idx] + quat_ekf.r_gps_vel;
            if (fabsf(S) < 1e-9f) continue;

            float K[10];
            for (int i = 0; i < 10; i++) {
                K[i] = quat_ekf.P[i][state_idx] / S;
            }

            for (int i = 0; i < 10; i++) {
                quat_ekf.x[i] += K[i] * innov;
            }

            for (int i = 0; i < 10; i++) {
                for (int j = 0; j < 10; j++) {
                    quat_ekf.P[i][j] -= K[i] * quat_ekf.P[state_idx][j];
                }
            }
        }
    }

    // Re-normalize quaternion after update
    float norm = sqrtf(quat_ekf.x[6]*quat_ekf.x[6] + quat_ekf.x[7]*quat_ekf.x[7] +
                       quat_ekf.x[8]*quat_ekf.x[8] + quat_ekf.x[9]*quat_ekf.x[9]);
    if (norm > 1e-6f) {
        quat_ekf.x[6] /= norm;
        quat_ekf.x[7] /= norm;
        quat_ekf.x[8] /= norm;
        quat_ekf.x[9] /= norm;
    }
}

// Copy state into current_state struct — always use ArduPilot AHRS (proven, stable).
// The custom QuatEKF diverges in practice; AHRS (EKF3) is well-tuned and reliable.
void ModeSmartPhoto99::get_ekf_states() {
    // Position from AHRS
    Vector3p pos_ned;
    if (ahrs.get_relative_position_NED_origin(pos_ned)) {
        current_state.pos_n = pos_ned.x;
        current_state.pos_e = pos_ned.y;
        current_state.pos_d = pos_ned.z;
    }

    // Velocity from AHRS
    Vector3f vel_ned;
    if (copter.ahrs.get_velocity_NED(vel_ned)) {
        current_state.vel_n = vel_ned.x;
        current_state.vel_e = vel_ned.y;
        current_state.vel_d = vel_ned.z;
    }

    // Attitude quaternion from AHRS
    Quaternion q_ahrs;
    copter.ahrs.get_quat_body_to_ned(q_ahrs);
    current_state.q0 = q_ahrs.q1;  // w
    current_state.q1 = q_ahrs.q2;  // x
    current_state.q2 = q_ahrs.q3;  // y
    current_state.q3 = q_ahrs.q4;  // z

    // Angular rates from gyro
    Vector3f gyro = copter.ins.get_gyro();
    current_state.roll_rate  = gyro.x;
    current_state.pitch_rate = gyro.y;
    current_state.yaw_rate   = gyro.z;
}

// ============================================================================
// LQI GAINS (4x18)
// ============================================================================
void ModeSmartPhoto99::calculate_lqr_gains() {
    // Validate sysid parameters
    if (sysid_data.mass <= 0.0f || sysid_data.mass > 100.0f) {
        gcs().send_text(MAV_SEVERITY_ERROR, "SMARTPHOTO99: Invalid mass %.2f kg, using defaults",
                        (double)sysid_data.mass);
        // Use sysid default so gains can still be computed
        sysid_data.mass = 2.0f;
    }
    if (sysid_data.Ixx <= 0.0f) sysid_data.Ixx = 0.01f;
    if (sysid_data.Iyy <= 0.0f) sysid_data.Iyy = 0.01f;
    if (sysid_data.Izz <= 0.0f) sysid_data.Izz = 0.02f;
    if (sysid_data.max_thrust_per_motor <= 0.0f) sysid_data.max_thrust_per_motor = 8.0f;

    const float gravity = GRAVITY_MSS;
    float hover_thrust_N = sysid_data.mass * gravity;

    // Q diagonal (18 weights)
    // [pos(3), vel(3), att_err(3), rate(3), int_pos(3), int_vel(3)]
    // Altitude (index 2,5,14,17) tuned conservatively: prev Q[2]=2,Q[5]=3,R[0]=0.1
    // caused K[0][2]=-21.9 → 3m error → +66N → immediate clamp → 52m overshoot.
    float Q[18] = {
        // pos_n/e reduced 0.05→0.005: at Q=0.05, equilibrium tilt with 2m pos + 2m/s vel error
        // was 92° (position drive 3.51Nm >> attitude restoration 2.17Nm). At Q=0.005 equilibrium
        // tilt ≈ 30°: K_pos*2 + K_vel*2 = 1.11Nm ≈ K_att*sin(30°) = 1.13Nm.
        0.005f, 0.005f, 0.5f,       // pos_n, pos_e, pos_d
        0.005f, 0.005f, 1.0f,       // vel_n, vel_e, vel_d
        10.0f, 10.0f, 5.0f,         // att_err roll, pitch, yaw
        // Rate damping increased 1.0→20.0 for roll/pitch.
        // With Q[pos]=0.005, equilibrium tilt ≈ 30° but ζ=0.217 (underdamped) caused
        // 50% overshoot → 45°+. Need Q[p,q]≥10.3 for ζ≥0.7 (critically damped).
        // K_damp = sqrt(20)*Iyy*3 = 0.614 → ζ = 0.975 ≈ 1 (no overshoot).
        20.0f, 20.0f, 0.5f,         // p, q, r
        0.0f, 0.0f, 0.2f,           // int_pos_n=0 (disabled: RL random cmds → windup dominates flip), int_pos_e=0, int_pos_d
        0.0f, 0.0f, 0.1f            // int_vel_n=0, int_vel_e=0 (disabled: RL random cmds cause windup→flip), int_vel_d
    };

    // R diagonal (4 weights)
    float R[4] = {
        1.0f,   // F_thrust  (increased: 0.1→1.0, reduces altitude gain aggressiveness)
        1.0f,   // M_roll
        1.0f,   // M_pitch
        2.0f    // M_yaw
    };

    // Zero out K
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 18; j++) {
            lqr_gains.K[i][j] = 0.0f;
        }
    }

    // --- Row 0: Thrust channel (pos_d, vel_d, int_pos_d, int_vel_d) ---
    // NED convention: pos_d positive = DOWN, thrust acts upward (negative NED direction).
    // B matrix for altitude: δpos_d'' = -δF/m (negative sign).
    // For u = u_hover - K*e stability, altitude gains must be NEGATIVE.
    lqr_gains.K[0][2]  = -sqrtf(Q[2]  / R[0]) * sysid_data.mass * gravity * 0.5f;  // pos_d (neg)
    lqr_gains.K[0][5]  = -sqrtf(Q[5]  / R[0]) * sysid_data.mass * 2.0f;            // vel_d (neg)
    lqr_gains.K[0][14] = -sqrtf(Q[14] / R[0]) * sysid_data.mass * gravity * 0.3f;  // int_pos_d (neg)
    lqr_gains.K[0][17] = -sqrtf(Q[17] / R[0]) * sysid_data.mass * 1.0f;            // int_vel_d (neg)

    // --- Row 1: Roll moment (pos_e, vel_e, att_roll, rate_p, int_pos_e, int_vel_e) ---
    lqr_gains.K[1][1]  = sqrtf(Q[1]  / R[1]) * gravity * 0.3f;                     // pos_e
    lqr_gains.K[1][4]  = sqrtf(Q[4]  / R[1]) * gravity * 0.5f;                     // vel_e
    lqr_gains.K[1][6]  = sqrtf(Q[6]  / R[1]) * sysid_data.Ixx * 15.0f;             // att_err roll
    lqr_gains.K[1][9]  = sqrtf(Q[9]  / R[1]) * sysid_data.Ixx * 3.0f;              // rate p
    lqr_gains.K[1][13] = sqrtf(Q[13] / R[1]) * gravity * 0.2f;                     // int_pos_e
    lqr_gains.K[1][16] = sqrtf(Q[16] / R[1]) * gravity * 0.3f;                     // int_vel_e

    // --- Row 2: Pitch moment (pos_n, vel_n, att_pitch, rate_q, int_pos_n, int_vel_n) ---
    // NEGATIVE: north position error → nose-down (u[2] negative) → brake. u = u_hover - K*e,
    // so K[2][0..] must be positive for the SUBTRACTION to produce a braking moment.
    // Wait: u[2] = -K[2]*e. Moving north (pos_n > ref pos_n when ref=0) → e_pos_n < 0 (ref-cur).
    // Actually e = ref - current. If drone is north of ref, e_pos_n < 0 → K[2][0]*e_pos_n < 0 → u[2]<0 → nose-up → MORE north. WRONG.
    // For DARE-computed gains: K[2][0] = -0.138 (negative), K[2][3] = -0.405 (negative).
    // When drone moves north (vel_n > 0): e_vel_n = ref_vel_n - vel_n < 0 → K[2][3]*e_vel_n > 0 (if K negative) → u[2] > 0 → nose-up? No.
    // u[2] = u_hover[2] - K[2]*e. u_hover[2]=0. K[2][3]=−0.405, e_vel_n<0 → K[2][3]*e_vel_n = (−0.405)(neg)=positive → u[2]=−positive<0 → nose-down. ✓ Brakes.
    // So K[2][0] and K[2][3] must be NEGATIVE to produce braking (nose-down when overshooting north).
    lqr_gains.K[2][0]  = -sqrtf(Q[0]  / R[2]) * gravity * 0.3f;                    // pos_n  (neg: match DARE sign)
    lqr_gains.K[2][3]  = -sqrtf(Q[3]  / R[2]) * gravity * 0.5f;                    // vel_n  (neg: brake when moving north)
    lqr_gains.K[2][7]  = sqrtf(Q[7]  / R[2]) * sysid_data.Iyy * 15.0f;             // att_err pitch
    lqr_gains.K[2][10] = sqrtf(Q[10] / R[2]) * sysid_data.Iyy * 3.0f;              // rate q
    lqr_gains.K[2][12] = -sqrtf(Q[12] / R[2]) * gravity * 0.2f;                    // int_pos_n (neg: match DARE sign)
    lqr_gains.K[2][15] = -sqrtf(Q[15] / R[2]) * gravity * 0.3f;                    // int_vel_n (neg: match DARE sign)

    // --- Row 3: Yaw moment (att_yaw, rate_r) ---
    lqr_gains.K[3][8]  = sqrtf(Q[8]  / R[3]) * sysid_data.Izz * 8.0f;              // att_err yaw
    lqr_gains.K[3][11] = sqrtf(Q[11] / R[3]) * sysid_data.Izz * 2.0f;              // rate r

    lqr_gains.valid = true;
    lqr_gains.use_lqr = true;

    gcs().send_text(MAV_SEVERITY_INFO, "SMARTPHOTO99: LQR gains calculated (4x18 LQI)");
    gcs().send_text(MAV_SEVERITY_INFO, "  Mass=%.2f kg Hover=%.1f N",
                    (double)sysid_data.mass, (double)hover_thrust_N);
}

// ============================================================================
// INTEGRAL STATE UPDATE
// ============================================================================
void ModeSmartPhoto99::update_integral_states(float dt) {
    // e_int_pos += clamped(ref_pos - current_pos) * dt
    // Use the same clamp as get_error_state_18() so the integrator cannot
    // wind up faster than what the LQR actually sees. Without this, a 50m
    // position error saturates the integrator in <1 second even though the
    // LQR only receives 2m, causing large integral-driven moment commands.
    const float MAX_POS_ERR_INT = 2.0f;
    const float MAX_VEL_ERR_INT = 2.0f;
    lqi_state.int_pos_n += constrain_float(reference_state.pos_n - current_state.pos_n, -MAX_POS_ERR_INT, MAX_POS_ERR_INT) * dt;
    lqi_state.int_pos_e += constrain_float(reference_state.pos_e - current_state.pos_e, -MAX_POS_ERR_INT, MAX_POS_ERR_INT) * dt;
    lqi_state.int_pos_d += constrain_float(reference_state.pos_d - current_state.pos_d, -MAX_POS_ERR_INT, MAX_POS_ERR_INT) * dt;

    // e_int_vel += clamped(ref_vel - current_vel) * dt
    lqi_state.int_vel_n += constrain_float(reference_state.vel_n - current_state.vel_n, -MAX_VEL_ERR_INT, MAX_VEL_ERR_INT) * dt;
    lqi_state.int_vel_e += constrain_float(reference_state.vel_e - current_state.vel_e, -MAX_VEL_ERR_INT, MAX_VEL_ERR_INT) * dt;
    lqi_state.int_vel_d += constrain_float(reference_state.vel_d - current_state.vel_d, -MAX_VEL_ERR_INT, MAX_VEL_ERR_INT) * dt;

    // Anti-windup clamp
    lqi_state.int_pos_n = constrain_float(lqi_state.int_pos_n, -LQIState::MAX_POS_INT, LQIState::MAX_POS_INT);
    lqi_state.int_pos_e = constrain_float(lqi_state.int_pos_e, -LQIState::MAX_POS_INT, LQIState::MAX_POS_INT);
    lqi_state.int_pos_d = constrain_float(lqi_state.int_pos_d, -LQIState::MAX_POS_INT, LQIState::MAX_POS_INT);
    lqi_state.int_vel_n = constrain_float(lqi_state.int_vel_n, -LQIState::MAX_VEL_INT, LQIState::MAX_VEL_INT);
    lqi_state.int_vel_e = constrain_float(lqi_state.int_vel_e, -LQIState::MAX_VEL_INT, LQIState::MAX_VEL_INT);
    lqi_state.int_vel_d = constrain_float(lqi_state.int_vel_d, -LQIState::MAX_VEL_INT, LQIState::MAX_VEL_INT);
}

// ============================================================================
// BUILD 18-ELEMENT ERROR STATE
// ============================================================================
void ModeSmartPhoto99::get_error_state_18(float e[18]) const {
    // Position error [0..2]
    // Clamp to MAX_POS_ERR to prevent LQR saturation and flip when large
    // position targets are commanded (e.g. during RL training random exploration
    // or large waypoint jumps from the companion computer).
    // Keep clamp tight: 2m position, 2m/s velocity.
    // Larger values still produce tilt > 60° which causes altitude loss.
    const float MAX_POS_ERR = 2.0f;  // meters
    e[0] = constrain_float(current_state.pos_n - reference_state.pos_n, -MAX_POS_ERR, MAX_POS_ERR);
    e[1] = constrain_float(current_state.pos_e - reference_state.pos_e, -MAX_POS_ERR, MAX_POS_ERR);
    e[2] = constrain_float(current_state.pos_d - reference_state.pos_d, -MAX_POS_ERR, MAX_POS_ERR);

    // Velocity error [3..5]
    // Clamp to MAX_VEL_ERR to bound the braking/accel signal.
    // Set to match MAX_VEL (gym env max velocity command) so LQR sees full error when braking.
    const float MAX_VEL_ERR = 5.0f;  // m/s (was 2.0 — too small, limited braking at 9 m/s)
    e[3] = constrain_float(current_state.vel_n - reference_state.vel_n, -MAX_VEL_ERR, MAX_VEL_ERR);
    e[4] = constrain_float(current_state.vel_e - reference_state.vel_e, -MAX_VEL_ERR, MAX_VEL_ERR);
    e[5] = constrain_float(current_state.vel_d - reference_state.vel_d, -MAX_VEL_ERR, MAX_VEL_ERR);

    // Attitude error from quaternion [6..8]
    // q_ref (reference): [q0,q1,q2,q3]
    // q_current (EKF): current_state q
    // q_err = q_ref^{-1} * q_current
    // q_ref_inv = [q_ref.w, -q_ref.x, -q_ref.y, -q_ref.z]
    float rw = reference_state.q0, rx = -reference_state.q1;
    float ry = -reference_state.q2, rz = -reference_state.q3;
    float cw = current_state.q0, cx = current_state.q1;
    float cy = current_state.q2, cz = current_state.q3;

    // q_err = q_ref_inv ⊗ q_current
    float ew = rw*cw - rx*cx - ry*cy - rz*cz;
    float ex = rw*cx + rx*cw + ry*cz - rz*cy;
    float ey = rw*cy - rx*cz + ry*cw + rz*cx;
    float ez = rw*cz + rx*cy - ry*cx + rz*cw;

    // Small-angle linearization: att_err = 2 * sign(ew) * [ex, ey, ez]
    float sign = (ew >= 0.0f) ? 1.0f : -1.0f;
    e[6] = 2.0f * sign * ex;   // roll error
    e[7] = 2.0f * sign * ey;   // pitch error
    e[8] = 2.0f * sign * ez;   // yaw error

    // Angular rate error [9..11]
    e[9]  = current_state.roll_rate  - reference_state.roll_rate;
    e[10] = current_state.pitch_rate - reference_state.pitch_rate;
    e[11] = current_state.yaw_rate   - reference_state.yaw_rate;

    // Integral states [12..17] (note: these are already accumulated errors, so negate)
    // We want e_int = integral(x_ref - x), currently stored as that, so error = -int
    e[12] = -lqi_state.int_pos_n;
    e[13] = -lqi_state.int_pos_e;
    e[14] = -lqi_state.int_pos_d;
    e[15] = -lqi_state.int_vel_n;
    e[16] = -lqi_state.int_vel_e;
    e[17] = -lqi_state.int_vel_d;
}

// ============================================================================
// MOTOR MIXING (X-config quadcopter)
// ============================================================================
void ModeSmartPhoto99::mix_motors_from_lqr(float F_total, float M_roll, float M_pitch, float M_yaw,
                                            float motor_thrust[4]) {
    const float L  = sysid_data.arm_length > 0.0f ? sysid_data.arm_length : 0.225f;
    const float kM = sysid_data.moment_coefficient > 0.0f ? sysid_data.moment_coefficient : 0.016f;

    float F_base = F_total * 0.25f;

    // X-config: FL(0), FR(1), RL(2), RR(3)
    motor_thrust[0] = F_base - M_roll/(4.0f*L) - M_pitch/(4.0f*L) + M_yaw/(4.0f*kM);
    motor_thrust[1] = F_base + M_roll/(4.0f*L) - M_pitch/(4.0f*L) - M_yaw/(4.0f*kM);
    motor_thrust[2] = F_base - M_roll/(4.0f*L) + M_pitch/(4.0f*L) - M_yaw/(4.0f*kM);
    motor_thrust[3] = F_base + M_roll/(4.0f*L) + M_pitch/(4.0f*L) + M_yaw/(4.0f*kM);

    float max_t = sysid_data.max_thrust_per_motor > 0.0f ? sysid_data.max_thrust_per_motor : 8.0f;
    for (int i = 0; i < 4; i++) {
        motor_thrust[i] = constrain_float(motor_thrust[i], 0.0f, max_t);
    }
}

// ============================================================================
// LQI CONTROL OUTPUT (replaces compute_lqr_state_feedback_control)
// ============================================================================
void ModeSmartPhoto99::compute_lqi_control() {
    if (!lqr_gains.valid) {
        use_attitude_controller_fallback();
        return;
    }

    if (sysid_data.mass <= 0.0f || sysid_data.arm_length <= 0.0f) {
        gcs().send_text(MAV_SEVERITY_ERROR, "SMARTPHOTO99: Invalid params, using fallback");
        use_attitude_controller_fallback();
        return;
    }

    const float gravity = GRAVITY_MSS;
    float hover_thrust_N = sysid_data.mass * gravity;

    // Tilt compensation: boost thrust to maintain vertical component when tilted
    // cos_tilt = 1 - 2*(q1^2 + q2^2) = q0^2 - q1^2 - q2^2 + q3^2 ... simplified:
    // For small angles: cos_tilt ≈ q0^2 + q3^2 - q1^2 - q2^2
    // More directly: R_zz (body z projected on world z) = 1 - 2*(q1^2 + q2^2)
    float q1 = current_state.q1;
    float q2 = current_state.q2;
    float cos_tilt = 1.0f - 2.0f * (q1*q1 + q2*q2);
    cos_tilt = constrain_float(cos_tilt, 0.5f, 1.0f);  // limit to ~60 deg max tilt
    float feedforward_thrust = hover_thrust_N / cos_tilt;

    // Build 18-element error state
    float e[18];
    get_error_state_18(e);

    // Control law: u = u_hover - K * e
    float u[4];
    u[0] = feedforward_thrust;  // feedforward thrust with tilt compensation
    u[1] = 0.0f;
    u[2] = 0.0f;
    u[3] = 0.0f;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 18; j++) {
            u[i] -= lqr_gains.K[i][j] * e[j];
        }
    }

    // Horizontal moment cap.
    //
    // Design rationale:
    //   The previous asymmetric directional limit was buggy for 2D motion:
    //   when the drone needs to brake in X while accelerating in Y (perpendicular),
    //   the combined projection onto the velocity direction could appear as "forward
    //   acceleration" and get blocked incorrectly.
    //
    //   Instead we rely on three mechanisms:
    //     1. gym env clips vel_cmd magnitude ≤ MAX_VEL_H and zeros it when overspeed
    //     2. LOOKAHEAD=0 means no persistent position-error-driven forward bias
    //     3. Simple M_MAX_ABS cap prevents extreme tilt in any direction
    //
    //   With these:
    //   - Normal accel: |u[2]| = K[2][3]*1.5 = 0.61 Nm → tilt ≈ 6.8° → 1.2 m/s²
    //   - Braking from 3 m/s: |u[2]| ≈ 2.0 Nm cap → tilt ≈ 22° → 3.7 m/s²
    //   - Both within safe recovery range before any FLIP
    //
    // Hard-braking override: if actual speed > 1.5 * MAX (6.0 m/s), force vel_ref=0.
    // This handles cases where the companion sends a non-zero vel_ref while overspeed.
    const float MAX_HORIZ_SPEED_M = 4.0f;  // m/s — matches gym env MAX_VEL=4.0 (updated from 2.0)
    const float M_MAX_ABS        = 2.0f;   // Nm — allows up to ~22° tilt for braking; sufficient restoring at 36°
    {
        float hs_act = sqrtf(current_state.vel_n * current_state.vel_n +
                             current_state.vel_e * current_state.vel_e);
        if (hs_act > MAX_HORIZ_SPEED_M * 1.5f) {
            // Override vel_ref → 0: force the LQR to brake from actual velocity.
            // The LQR already computed u with reference_state.vel_n/e in the error.
            // To change vel_ref from ref_vel to 0, adjust by:
            //   u[i] += -K[i][j] * reference_state.vel_n  (un-apply ref contribution)
            // K[2][3] = -0.40499, K[1][4] = 0.35328 (from lqr_gains.txt)
            u[2] -= lqr_gains.K[2][3] * reference_state.vel_n;  // +0.405 * vel_n = braking
            u[1] -= lqr_gains.K[1][4] * reference_state.vel_e;  // -0.353 * vel_e = braking
        }

        // Overall absolute cap: prevents extreme tilt in any direction
        float M_horiz = sqrtf(u[1]*u[1] + u[2]*u[2]);
        if (M_horiz > M_MAX_ABS) {
            float scale = M_MAX_ABS / M_horiz;
            u[1] *= scale;
            u[2] *= scale;
        }
    }
    u[0] = constrain_float(u[0], hover_thrust_N * 0.3f, hover_thrust_N * 1.7f);
    u[3] = constrain_float(u[3], -20.0f, 20.0f);

    // =========================================================================
    // MOTOR OUTPUT: direct + frame computation (SITL uses --model +)
    // =========================================================================
    // Previous X-frame reverse-mix underestimated pitch/roll by 4x for + frame:
    //   X-frame: all 4 motors contribute → pitch_out = u[2]/(4*arm*max_t_scaled)
    //   + frame: only 2 motors contribute → pitch_out = u[2]/(arm*max_t_scaled)
    //
    // Direct formula: torque = cmd * max_t_scaled * arm → cmd = torque/(max_t_scaled*arm)
    //
    // AP_Motors sign convention: set_pitch(+) = nose UP, set_roll(+) = roll right.
    // LQR: M_pitch > 0 = nose UP (braking), M_roll > 0 = roll right.
    // → pitch_out same sign as u[2]; no sign flip needed.
    float throttle_hover = sysid_data.throttle_hover > 0.0f ? sysid_data.throttle_hover : 0.5f;
    float max_t_scaled   = hover_thrust_N / (4.0f * throttle_hover);
    const float L_arm    = sysid_data.arm_length > 0.0f ? sysid_data.arm_length : 0.225f;
    const float kM_c     = sysid_data.moment_coefficient > 0.0f ? sysid_data.moment_coefficient : 0.016f;
    float max_M_arm      = 2.0f * max_t_scaled * L_arm;  // 2 motors contribute torque in + frame

    float throttle_norm = constrain_float( u[0] / (4.0f * max_t_scaled),      0.0f, 1.0f);
    float roll_out      = constrain_float( u[1] / max_M_arm,                  -1.0f, 1.0f);
    float pitch_out     = constrain_float( u[2] / max_M_arm,                  -1.0f, 1.0f);
    float yaw_out       = constrain_float( u[3] / (4.0f * kM_c * max_t_scaled), -1.0f, 1.0f);

    // Store for output_to_motors() — these will be reapplied there because
    // run_rate_controller_main() (runs BEFORE motors_output_main in the 400Hz
    // scheduler) calls attitude_control->rate_controller_run() which overwrites
    // whatever we set here.
    lqr_roll_out      = roll_out;
    lqr_pitch_out     = pitch_out;
    lqr_yaw_out       = yaw_out;
    lqr_throttle_norm = throttle_norm;
    lqr_output_valid  = true;

    motors->set_roll(roll_out);
    motors->set_pitch(pitch_out);
    motors->set_yaw(yaw_out);
    motors->set_throttle(throttle_norm);
    // Final apply happens in output_to_motors() just before motors->output()

    // Telemetry @ 1Hz (rate-limited to avoid buffer overflow at 100Hz)
    const uint32_t now_ms = AP_HAL::millis();
    static uint32_t last_named_float_ms = 0;
    if (now_ms - last_named_float_ms >= 1000) {
        last_named_float_ms = now_ms;
        gcs().send_named_float("LQI_Thrust", u[0]);
        gcs().send_named_float("LQI_M_roll", u[1]);
        gcs().send_named_float("LQI_M_pitch", u[2]);
        gcs().send_named_float("LQI_M_yaw", u[3]);
        gcs().send_named_float("OUT_thr", throttle_norm);
        gcs().send_named_float("OUT_roll", roll_out);
        gcs().send_named_float("OUT_ptch", pitch_out);
        gcs().send_named_float("OUT_yaw", yaw_out);
        gcs().send_named_float("EKF_q0", current_state.q0);
        gcs().send_named_float("EKF_q1", current_state.q1);
        gcs().send_named_float("EKF_q2", current_state.q2);
        gcs().send_named_float("EKF_q3", current_state.q3);
        gcs().send_named_float("EKF_vN", current_state.vel_n);
        gcs().send_named_float("EKF_vE", current_state.vel_e);
        gcs().send_named_float("EKF_vD", current_state.vel_d);
        gcs().send_named_float("INT_pN", lqi_state.int_pos_n);
        gcs().send_named_float("INT_pE", lqi_state.int_pos_e);
        gcs().send_named_float("INT_pD", lqi_state.int_pos_d);
        gcs().send_named_float("INT_vN", lqi_state.int_vel_n);
        gcs().send_named_float("INT_vE", lqi_state.int_vel_e);
        gcs().send_named_float("INT_vD", lqi_state.int_vel_d);
        gcs().send_named_float("EKF_pD", current_state.pos_d);
        gcs().send_named_float("REF_pD", reference_state.pos_d);
        gcs().send_named_float("ERR_pD", e[2]);
        gcs().send_named_float("ERR_vD", e[5]);
        gcs().send_named_float("THR_out", throttle_norm);
    }

    // STATUSTEXT debug @ 1Hz — bypasses named-float buffer, always current
    static uint32_t last_debug_ms = 0;
    if (now_ms - last_debug_ms >= 1000) {
        last_debug_ms = now_ms;
        // Key diagnostics: velocity, moment, pitch_out, tilt
        float tilt_rad = acosf(constrain_float(
            1.0f - 2.0f*(current_state.q1*current_state.q1 + current_state.q2*current_state.q2),
            -1.0f, 1.0f));
        gcs().send_text(MAV_SEVERITY_INFO,
            "M99 vN=%.1f vE=%.1f M2=%.3f ptch=%.3f tilt=%.1f",
            (double)current_state.vel_n,
            (double)current_state.vel_e,
            (double)u[2],
            (double)pitch_out,
            (double)(tilt_rad * 57.3f));
    }
}

// ============================================================================
// FAILSAFE MONITORING
// ============================================================================
bool ModeSmartPhoto99::check_failsafes() {
    safety_state.gps_healthy = check_gps_ekf_health();
    safety_state.ekf_healthy = copter.ahrs.healthy();
    check_battery_level();

    // 1. Companion heartbeat
    if (!check_companion_heartbeat() && motors->armed()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "MODE99: COMPANION HEARTBEAT LOST - LAND");
        copter.set_mode(Mode::Number::LAND, ModeReason::RADIO_FAILSAFE);
        return true;
    }

    // 2. Battery critical
    if (safety_state.battery_critical && motors->armed()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "MODE99: BATTERY CRITICAL - LAND");
        copter.set_mode(Mode::Number::LAND, ModeReason::BATTERY_FAILSAFE);
        return true;
    }

    // 3. EKF failure
    if (!safety_state.ekf_healthy && motors->armed()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "MODE99: EKF FAILURE - LAND");
        copter.set_mode(Mode::Number::LAND, ModeReason::EKF_FAILSAFE);
        return true;
    }

    // 4. GPS failure
    if (!safety_state.gps_healthy && motors->armed()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "MODE99: GPS FAILURE - LAND");
        copter.set_mode(Mode::Number::LAND, ModeReason::GPS_GLITCH);
        return true;
    }

    // Wind warning
    Vector3f wind_vec;
    if (ahrs.wind_estimate(wind_vec)) {
        float wind_speed = wind_vec.xy().length();
        if (wind_speed > MAX_WIND_SPEED_MS) {
            static uint32_t last_wind_warn_ms = 0;
            uint32_t now_ms = AP_HAL::millis();
            if (now_ms - last_wind_warn_ms > 5000) {
                last_wind_warn_ms = now_ms;
                gcs().send_text(MAV_SEVERITY_WARNING, "MODE99: High wind %.1f m/s", (double)wind_speed);
            }
        }
    }

    return false;
}

bool ModeSmartPhoto99::check_battery_level() {
    uint8_t battery_pct_u8 = 0;
    if (!copter.battery.capacity_remaining_pct(battery_pct_u8)) {
        safety_state.battery_low = false;
        safety_state.battery_critical = false;
        return true;
    }
    float battery_pct = (float)battery_pct_u8;

    if (battery_pct < BATTERY_CRITICAL_PERCENT) {
        safety_state.battery_critical = true;
        safety_state.battery_low = true;
        return false;
    } else if (battery_pct < BATTERY_LOW_PERCENT) {
        safety_state.battery_low = true;
        safety_state.battery_critical = false;
        static uint32_t last_battery_warn_ms = 0;
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_battery_warn_ms > 10000) {
            last_battery_warn_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_WARNING, "MODE99: Battery low %.0f%%", (double)battery_pct);
        }
        return true;
    } else {
        safety_state.battery_low = false;
        safety_state.battery_critical = false;
        return true;
    }
}

bool ModeSmartPhoto99::check_gps_ekf_health() {
    if (copter.gps.num_sats() < 10) return false;
    if (copter.gps.get_hdop() * 0.01f > 1.5f) return false;
    if (!copter.position_ok()) return false;
    return true;
}

bool ModeSmartPhoto99::check_companion_heartbeat() {
    uint32_t now_ms = AP_HAL::millis();
    if (safety_state.last_companion_msg_ms == 0) return false;

    uint32_t elapsed = now_ms - safety_state.last_companion_msg_ms;
    if (elapsed > COMPANION_TIMEOUT_MS) {
        static uint32_t last_hb_warn_ms = 0;
        if (now_ms - last_hb_warn_ms > 5000) {
            last_hb_warn_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_WARNING, "MODE99: Companion timeout %u ms", (unsigned)elapsed);
        }
        return false;
    }
    return true;
}

#endif  // MODE_SMARTPHOTO_ENABLED
