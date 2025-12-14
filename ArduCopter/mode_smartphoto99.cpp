// ArduCopter/mode_smartphoto99.cpp
// Smart Photo Mode (Mode 99) - EKF State Feedback Control
//
// Control Architecture:
// - Main loop: Runs at 400Hz (main scheduler rate)
// - EKF State Feedback Control: Runs at 100Hz (10ms period)
// - Wind Data Transmission: Runs at 100Hz (10ms period)
// - Attitude Rate Controller: Runs at 400Hz (or faster on rate thread)
//
// The 100Hz control rate provides sufficient bandwidth for position/velocity
// control while reducing computational load compared to 400Hz execution.

#include "Copter.h"
#include "mode.h"

#if MODE_SMARTPHOTO_ENABLED

// Initialize mode 99 - Smart Photo mode with identified parameters
ModeSmartPhoto99::ModeSmartPhoto99() :
    target_alt_cm(0.0f)
{
    sysid_data.parameters_loaded = false;
    sysid_data.mass = 0.0f;
    sysid_data.Ixx = 0.0f;
    sysid_data.Iyy = 0.0f;
    sysid_data.Izz = 0.0f;
    sysid_data.motor_kv = 0.0f;
    sysid_data.max_thrust_per_motor = 0.0f;
    sysid_data.roll_rate_gain = 0.0f;
    sysid_data.pitch_rate_gain = 0.0f;
    sysid_data.yaw_rate_gain = 0.0f;
    sysid_data.throttle_hover = 0.0f;
    sysid_data.sample_count = 0;

    // Initialize state feedback gains
    control_gains.gains_valid = false;
    for (int i = 0; i < 3; i++) {
        control_gains.K_pos[i] = 0.0f;
        control_gains.K_vel[i] = 0.0f;
        control_gains.K_att[i] = 0.0f;
        control_gains.K_rate[i] = 0.0f;
    }

    // Initialize state vectors
    memset(&current_state, 0, sizeof(StateVector));
    memset(&reference_state, 0, sizeof(StateVector));

    target_position_ne.zero();
    target_altitude = 0.0f;
    target_yaw = 0.0f;

    // Initialize companion command structure
    companion_cmd.position_ned.zero();
    companion_cmd.velocity_ned.zero();
    companion_cmd.yaw = 0.0f;
    companion_cmd.yaw_rate = 0.0f;
    companion_cmd.timestamp_ms = 0;
    companion_cmd.valid = false;

    // Initialize attitude target structure
    attitude_target.roll = 0.0f;
    attitude_target.pitch = 0.0f;
    attitude_target.yaw = 0.0f;
    attitude_target.roll_rate = 0.0f;
    attitude_target.pitch_rate = 0.0f;
    attitude_target.yaw_rate = 0.0f;
    attitude_target.roll_prev = 0.0f;
    attitude_target.pitch_prev = 0.0f;
    attitude_target.yaw_prev = 0.0f;
    attitude_target.last_update_ms = 0;

    // Initialize smoothing parameters
    smoothing.max_tilt_rate = 1.0f;      // 1 rad/s (~57 deg/s) max tilt rate
    smoothing.max_yaw_rate = 0.5f;       // 0.5 rad/s (~28 deg/s) max yaw rate
    smoothing.attitude_tc = 0.15f;       // 150ms time constant for smoothing
    smoothing.use_companion_cmd = false; // Start with pilot input, enable via command

    // Initialize 100Hz timing control
    last_state_feedback_ms = 0;
    last_wind_send_ms = 0;
    state_feedback_counter = 0;
    wind_send_counter = 0;
}

// Initialize Smart Photo mode
bool ModeSmartPhoto99::init(bool ignore_checks) {
    // Check position estimate
    if (!copter.position_ok() && !ignore_checks) {
        return false;
    }

    // Load identified parameters from file
    if (!load_identified_parameters()) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "SMARTPHOTO: No sysid parameters found, using defaults");
    } else {
        // Calculate state feedback gains from identified parameters
        calculate_state_feedback_gains();
        apply_identified_parameters();
    }

    // Get initial EKF states
    get_ekf_states();

    // Set initial reference state to current state
    reference_state = current_state;
    target_altitude = current_state.pos_d;
    target_yaw = current_state.yaw;
    target_position_ne.x = current_state.pos_n;
    target_position_ne.y = current_state.pos_e;

    // Initialize attitude targets to current attitude for smooth startup
    attitude_target.roll = current_state.roll;
    attitude_target.pitch = current_state.pitch;
    attitude_target.yaw = current_state.yaw;
    attitude_target.roll_prev = current_state.roll;
    attitude_target.pitch_prev = current_state.pitch;
    attitude_target.yaw_prev = current_state.yaw;
    attitude_target.roll_rate = 0.0f;
    attitude_target.pitch_rate = 0.0f;
    attitude_target.yaw_rate = 0.0f;
    attitude_target.last_update_ms = AP_HAL::millis();

    // Initialize 100Hz timing to start immediately
    uint32_t now_ms = AP_HAL::millis();
    last_state_feedback_ms = now_ms;
    last_wind_send_ms = now_ms;

    gcs().send_text(MAV_SEVERITY_INFO, "SMARTPHOTO99: EKF State Feedback @ 100Hz");
    gcs().send_text(MAV_SEVERITY_INFO, "SMARTPHOTO99: Wind Telemetry @ 100Hz");

    return true;
}

// Run Smart Photo mode - EKF State Feedback Control at 100Hz
void ModeSmartPhoto99::run() {
    // Apply motor interlock (enable motors if armed)
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Get current time for 100Hz scheduling
    const uint32_t now_ms = AP_HAL::millis();

    // Always get current EKF states (needed for all loops)
    get_ekf_states();

    // ==========================================
    // WIND DATA TRANSMISSION @ 100Hz
    // Send 3D wind speed to companion computer at 100Hz
    // ==========================================
    if (now_ms - last_wind_send_ms >= WIND_SEND_DT_MS) {
        last_wind_send_ms = now_ms;
        wind_send_counter++;

        Vector3f wind_vec;
        if (ahrs.wind_estimate(wind_vec)) {
            // Wind estimate available from EKF
            // wind_vec is in NED frame (North, East, Down) in m/s
            float wind_speed = wind_vec.xy().length();            // Horizontal wind speed (m/s)
            float wind_dir = atan2f(wind_vec.y, wind_vec.x);      // Wind direction (radians, 0 = from North)

            // Send 3D wind data to companion computer @ 100Hz
            // All units: m/s for components and magnitude, radians for direction
            gcs().send_named_float("WindSpd", wind_speed);        // Horizontal magnitude [m/s]
            gcs().send_named_float("WindDir", wind_dir);          // Direction [radians], 0 = North
            gcs().send_named_float("WindN", wind_vec.x);          // North component [m/s]
            gcs().send_named_float("WindE", wind_vec.y);          // East component [m/s]
            gcs().send_named_float("WindD", wind_vec.z);          // Down component [m/s]
        }
    }

    // ==========================================
    // STATE FEEDBACK CONTROL @ 100Hz
    // EKF state feedback control runs at 100Hz for computational efficiency
    // ==========================================
    if (now_ms - last_state_feedback_ms >= STATE_FEEDBACK_DT_MS) {
        last_state_feedback_ms = now_ms;
        state_feedback_counter++;

        // Calculate dt for 100Hz loop (nominally 0.01s)
        const float dt_100hz = STATE_FEEDBACK_DT_MS * 0.001f;

        // Determine control source: companion computer or pilot input
        Vector3f target_velocity_ned;
        float target_yaw_cmd;
        float target_yaw_rate_cmd;

        if (smoothing.use_companion_cmd && companion_command_valid()) {
            // Use companion computer commands
            target_velocity_ned = companion_cmd.velocity_ned;
            target_yaw_cmd = companion_cmd.yaw;
            target_yaw_rate_cmd = companion_cmd.yaw_rate;

            // Update position targets from companion
            target_position_ne.x = companion_cmd.position_ned.x;
            target_position_ne.y = companion_cmd.position_ned.y;
            target_altitude = companion_cmd.position_ned.z;

            // Also set vertical velocity
            reference_state.vel_d = target_velocity_ned.z;
        } else {
            // Fallback to pilot input or timeout - disable companion mode
            if (smoothing.use_companion_cmd) {
                smoothing.use_companion_cmd = false;
                gcs().send_text(MAV_SEVERITY_WARNING, "SMARTPHOTO: Companion timeout, using pilot input");
            }

            // Get pilot desired climb rate
            float target_climb_rate_ms = get_pilot_desired_climb_rate_ms();
            target_climb_rate_ms = constrain_float(target_climb_rate_ms,
                -get_pilot_speed_dn_ms(), get_pilot_speed_up_ms());

            // Update target altitude (down is positive in NED)
            target_altitude -= target_climb_rate_ms * dt_100hz;
            reference_state.vel_d = -target_climb_rate_ms;

            // Get pilot desired horizontal velocity (from lean angle input)
            float target_roll_rad, target_pitch_rad;
            get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad,
                attitude_control->lean_angle_max_rad(),
                attitude_control->get_althold_lean_angle_max_rad());

            // Convert lean angles to velocity commands (simplified)
            float max_vel_xy = 2.0f;  // m/s
            target_velocity_ned.x = -target_pitch_rad * max_vel_xy / attitude_control->lean_angle_max_rad();
            target_velocity_ned.y = target_roll_rad * max_vel_xy / attitude_control->lean_angle_max_rad();
            target_velocity_ned.z = -target_climb_rate_ms;

            // Integrate velocity to position
            target_position_ne.x += target_velocity_ned.x * dt_100hz;
            target_position_ne.y += target_velocity_ned.y * dt_100hz;

            // Get pilot's desired yaw rate
            target_yaw_rate_cmd = get_pilot_desired_yaw_rate_rads();
            target_yaw += target_yaw_rate_cmd * dt_100hz;
            target_yaw = wrap_PI(target_yaw);
            target_yaw_cmd = target_yaw;
        }

        // Calculate desired attitude from velocity command
        float roll_desired, pitch_desired;
        calculate_desired_attitude_from_velocity(target_velocity_ned, roll_desired, pitch_desired);

        // Smooth attitude targets to ensure smooth motion
        smooth_attitude_targets(roll_desired, pitch_desired, target_yaw_cmd, dt_100hz);

        // Calculate attitude rates from smoothed attitude changes
        calculate_attitude_rates(dt_100hz);

        // Set reference state using smoothed attitude targets
        reference_state.pos_n = target_position_ne.x;
        reference_state.pos_e = target_position_ne.y;
        reference_state.pos_d = target_altitude;
        reference_state.vel_n = target_velocity_ned.x;
        reference_state.vel_e = target_velocity_ned.y;

        // Use smoothed attitude targets and rates
        reference_state.roll = attitude_target.roll;
        reference_state.pitch = attitude_target.pitch;
        reference_state.yaw = attitude_target.yaw;
        reference_state.roll_rate = attitude_target.roll_rate;
        reference_state.pitch_rate = attitude_target.pitch_rate;
        reference_state.yaw_rate = attitude_target.yaw_rate;

        // Compute state feedback control @ 100Hz
        compute_state_feedback_control();
    } else {
        // Between 100Hz updates: Still run attitude controller at main loop rate
        // This ensures smooth motor output even though state feedback runs at 100Hz
        // The attitude controller uses the last computed reference values

        // Re-apply the last computed attitude and throttle targets
        // This keeps the attitude controller updated at the main loop rate (400Hz)
        if (control_gains.gains_valid) {
            // Use the last computed state feedback control values
            // compute_state_feedback_control() already set the attitude controller targets
            // Just need to call the rate controller update at main loop rate
            attitude_control->rate_controller_run();
        } else {
            // Fallback mode between updates
            use_attitude_controller_fallback();
        }
    }
}

// Load identified parameters from file (saved by mode 98)
bool ModeSmartPhoto99::load_identified_parameters() {
    const char* filename = "sysid_params.txt";

    int fd = AP::FS().open(filename, O_RDONLY);
    if (fd == -1) {
        return false;
    }

    char buffer[512];
    ssize_t bytes_read = AP::FS().read(fd, buffer, sizeof(buffer) - 1);
    AP::FS().close(fd);

    if (bytes_read <= 0) {
        gcs().send_text(MAV_SEVERITY_ERROR, "SMARTPHOTO: Failed to read parameters");
        return false;
    }

    buffer[bytes_read] = '\0';

    // Parse parameters from file
    char* line = strtok(buffer, "\n");
    while (line != NULL) {
        // Skip comments and empty lines
        if (line[0] == '#' || line[0] == '\0' || line[0] == '\r') {
            line = strtok(NULL, "\n");
            continue;
        }

        // Parse key=value pairs
        char key[32];
        float value;
        if (sscanf(line, "%31[^=]=%f", key, &value) == 2) {
            if (strcmp(key, "MASS") == 0) {
                sysid_data.mass = value;
            } else if (strcmp(key, "IXX") == 0) {
                sysid_data.Ixx = value;
            } else if (strcmp(key, "IYY") == 0) {
                sysid_data.Iyy = value;
            } else if (strcmp(key, "IZZ") == 0) {
                sysid_data.Izz = value;
            } else if (strcmp(key, "MOTOR_KV") == 0) {
                sysid_data.motor_kv = value;
            } else if (strcmp(key, "MAX_THRUST") == 0) {
                sysid_data.max_thrust_per_motor = value;
            } else if (strcmp(key, "ROLL_GAIN") == 0) {
                sysid_data.roll_rate_gain = value;
            } else if (strcmp(key, "PITCH_GAIN") == 0) {
                sysid_data.pitch_rate_gain = value;
            } else if (strcmp(key, "YAW_GAIN") == 0) {
                sysid_data.yaw_rate_gain = value;
            } else if (strcmp(key, "THROTTLE_HOVER") == 0) {
                sysid_data.throttle_hover = value;
            } else if (strcmp(key, "SAMPLES") == 0) {
                sysid_data.sample_count = (uint32_t)value;
            }
        }

        line = strtok(NULL, "\n");
    }

    sysid_data.parameters_loaded = true;

    gcs().send_text(MAV_SEVERITY_INFO,
        "SMARTPHOTO: Loaded params Roll=%.2f Pitch=%.2f Yaw=%.2f THR=%.2f",
        sysid_data.roll_rate_gain,
        sysid_data.pitch_rate_gain,
        sysid_data.yaw_rate_gain,
        sysid_data.throttle_hover);

    return true;
}

// Apply identified parameters to the flight controller
void ModeSmartPhoto99::apply_identified_parameters() {
    if (!sysid_data.parameters_loaded) {
        return;
    }

    // Note: The identified parameters are loaded and available
    // They can be used by the controller if needed
    // Throttle hover and rate gains would normally be applied through
    // the attitude controller and motors interfaces
    // For demonstration, we just log that parameters are loaded

    gcs().send_text(MAV_SEVERITY_INFO,
        "SMARTPHOTO: Using identified parameters from mode 98");
    gcs().send_text(MAV_SEVERITY_INFO,
        "SMARTPHOTO: THR hover=%.3f", sysid_data.throttle_hover);
}

// Calculate state feedback gains from identified parameters
void ModeSmartPhoto99::calculate_state_feedback_gains() {
    if (!sysid_data.parameters_loaded) {
        return;
    }

    // Design state feedback gains based on identified system parameters
    // Using simplified LQR-like gains based on identified rate gains

    // Position gains (P-like)
    control_gains.K_pos[0] = 0.5f;  // North
    control_gains.K_pos[1] = 0.5f;  // East
    control_gains.K_pos[2] = 1.0f;  // Down (altitude)

    // Velocity gains (D-like)
    control_gains.K_vel[0] = 1.0f;  // North velocity
    control_gains.K_vel[1] = 1.0f;  // East velocity
    control_gains.K_vel[2] = 2.0f;  // Down velocity

    // Attitude gains - use identified parameters
    control_gains.K_att[0] = sysid_data.roll_rate_gain * 3.0f;   // Roll
    control_gains.K_att[1] = sysid_data.pitch_rate_gain * 3.0f;  // Pitch
    control_gains.K_att[2] = sysid_data.yaw_rate_gain * 2.0f;    // Yaw

    // Rate gains - directly from system ID
    control_gains.K_rate[0] = sysid_data.roll_rate_gain;
    control_gains.K_rate[1] = sysid_data.pitch_rate_gain;
    control_gains.K_rate[2] = sysid_data.yaw_rate_gain;

    control_gains.gains_valid = true;

    gcs().send_text(MAV_SEVERITY_INFO,
        "SMARTPHOTO: State feedback gains calculated");
    gcs().send_text(MAV_SEVERITY_INFO,
        "K_att: R=%.2f P=%.2f Y=%.2f",
        control_gains.K_att[0], control_gains.K_att[1], control_gains.K_att[2]);
}

// Get current states from EKF
void ModeSmartPhoto99::get_ekf_states() {
    // Get position (NED frame, meters)
    Vector3p pos_ned;
    if (!ahrs.get_relative_position_NED_origin(pos_ned)) {
        // If not available, use zero
        pos_ned.zero();
    }
    current_state.pos_n = pos_ned.x;
    current_state.pos_e = pos_ned.y;
    current_state.pos_d = pos_ned.z;

    // Get velocity (NED frame, m/s)
    Vector3f vel_ned;
    if (!copter.ahrs.get_velocity_NED(vel_ned)) {
        vel_ned.zero();
    }
    current_state.vel_n = vel_ned.x;
    current_state.vel_e = vel_ned.y;
    current_state.vel_d = vel_ned.z;

    // Get attitude (radians)
    current_state.roll = copter.ahrs.get_roll();
    current_state.pitch = copter.ahrs.get_pitch();
    current_state.yaw = copter.ahrs.get_yaw();

    // Get angular rates (rad/s)
    Vector3f gyro = copter.ahrs.get_gyro();
    current_state.roll_rate = gyro.x;
    current_state.pitch_rate = gyro.y;
    current_state.yaw_rate = gyro.z;
}

// Compute state feedback control law using EKF states
// Runs at 100Hz (10ms period) for computational efficiency
void ModeSmartPhoto99::compute_state_feedback_control() {
    // Note: This function is called at 100Hz from run()
    // attitude_control and pos_control use their own dt from the main loop (400Hz)
    // State feedback calculations are performed at 100Hz for efficiency

    // Use default gains if not loaded from system ID
    if (!control_gains.gains_valid) {
        // Fallback to default position control with attitude controller
        use_attitude_controller_fallback();
        return;
    }

    // ==========================================
    // OUTER LOOP: Position and Velocity Control
    // Runs at main loop rate (400Hz)
    // ==========================================

    // Calculate position errors (NED frame, meters)
    float pos_err_n = reference_state.pos_n - current_state.pos_n;
    float pos_err_e = reference_state.pos_e - current_state.pos_e;
    float pos_err_d = reference_state.pos_d - current_state.pos_d;

    // Calculate velocity errors (NED frame, m/s)
    float vel_err_n = reference_state.vel_n - current_state.vel_n;
    float vel_err_e = reference_state.vel_e - current_state.vel_e;
    float vel_err_d = reference_state.vel_d - current_state.vel_d;

    // State feedback control law: u = K_p * e_pos + K_d * e_vel
    // Outputs: desired horizontal accelerations (m/s²)
    float accel_cmd_n = control_gains.K_pos[0] * pos_err_n + control_gains.K_vel[0] * vel_err_n;
    float accel_cmd_e = control_gains.K_pos[1] * pos_err_e + control_gains.K_vel[1] * vel_err_e;
    float accel_cmd_d = control_gains.K_pos[2] * pos_err_d + control_gains.K_vel[2] * vel_err_d;

    // Limit acceleration commands for safety
    const float max_accel_xy = 5.0f;  // m/s² horizontal
    const float max_accel_z = 5.0f;   // m/s² vertical
    accel_cmd_n = constrain_float(accel_cmd_n, -max_accel_xy, max_accel_xy);
    accel_cmd_e = constrain_float(accel_cmd_e, -max_accel_xy, max_accel_xy);
    accel_cmd_d = constrain_float(accel_cmd_d, -max_accel_z, max_accel_z);

    // ==========================================
    // MIDDLE LOOP: Convert Accelerations to Attitude Targets
    // Using quadcopter dynamics: a = g * tan(θ)
    // For small angles: θ ≈ a / g
    // ==========================================

    const float gravity = GRAVITY_MSS;  // 9.80665 m/s² (ArduPilot constant)

    // Desired attitude from acceleration commands
    // Note: In NED frame, positive pitch tilts forward (north)
    //       positive roll tilts right (east)
    float desired_pitch = atanf(accel_cmd_n / gravity);
    float desired_roll = -atanf(accel_cmd_e / gravity);  // Negative for correct direction

    // Apply attitude limits
    const float max_tilt_rad = attitude_control->lean_angle_max_rad();
    desired_roll = constrain_float(desired_roll, -max_tilt_rad, max_tilt_rad);
    desired_pitch = constrain_float(desired_pitch, -max_tilt_rad, max_tilt_rad);

    // Vertical thrust command (normalized 0-1)
    // thrust = (m * (a_z + g)) / (max_thrust * cos(tilt))
    // Simplified: thrust_normalized = hover_throttle + delta_throttle
    float thrust_normalized = sysid_data.throttle_hover + (accel_cmd_d / gravity) * 0.1f;
    thrust_normalized = constrain_float(thrust_normalized, 0.1f, 0.9f);

    // ==========================================
    // INNER LOOP: Attitude Control with State Feedback
    // Calculate attitude errors and rates
    // ==========================================

    // Attitude errors (radians)
    float roll_err = desired_roll - current_state.roll;
    float pitch_err = desired_pitch - current_state.pitch;
    float yaw_err = wrap_PI(reference_state.yaw - current_state.yaw);

    // State feedback for attitude: rate_cmd = K_att * err_att + feedforward - K_rate * rate_current
    // This provides damping while tracking the reference attitude
    float roll_rate_cmd = control_gains.K_att[0] * roll_err
                         + reference_state.roll_rate
                         - control_gains.K_rate[0] * current_state.roll_rate;

    float pitch_rate_cmd = control_gains.K_att[1] * pitch_err
                          + reference_state.pitch_rate
                          - control_gains.K_rate[1] * current_state.pitch_rate;

    float yaw_rate_cmd = control_gains.K_att[2] * yaw_err
                        + reference_state.yaw_rate
                        - control_gains.K_rate[2] * current_state.yaw_rate;

    // Limit rate commands
    const float max_rate_rp = radians(200.0f);  // 200 deg/s for roll/pitch
    const float max_rate_yaw = radians(90.0f);  // 90 deg/s for yaw
    roll_rate_cmd = constrain_float(roll_rate_cmd, -max_rate_rp, max_rate_rp);
    pitch_rate_cmd = constrain_float(pitch_rate_cmd, -max_rate_rp, max_rate_rp);
    yaw_rate_cmd = constrain_float(yaw_rate_cmd, -max_rate_yaw, max_rate_yaw);

    // ==========================================
    // OUTPUT: Use ArduPilot's Attitude Controller
    // This ensures proper integration with motor mixing and safety features
    // ==========================================

    // Set attitude and rate targets for the attitude controller
    // The attitude controller will handle the rate loop and motor mixing
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
        desired_roll,
        desired_pitch,
        yaw_rate_cmd
    );

    // Set throttle output
    attitude_control->set_throttle_out(thrust_normalized, true, g.throttle_filt);

    // Telemetry for debugging/monitoring
    // All angles in radians, thrust normalized 0-1
    gcs().send_named_float("SF_RollErr", roll_err);          // [radians]
    gcs().send_named_float("SF_PitchErr", pitch_err);        // [radians]
    gcs().send_named_float("SF_YawErr", yaw_err);            // [radians]
    gcs().send_named_float("SF_ThrustCmd", thrust_normalized); // [0-1]

    // Report execution rate every second (100 executions @ 100Hz)
    if (state_feedback_counter % 100 == 0) {
        gcs().send_named_float("SF_Rate", 100.0f);  // Nominal 100Hz
        gcs().send_named_float("Wind_Rate", 100.0f); // Nominal 100Hz
    }
}

// Fallback controller using standard ArduPilot attitude control
void ModeSmartPhoto99::use_attitude_controller_fallback() {
    // Use standard position controller if available
    if (copter.position_ok()) {
        // Set position targets using the smoothed attitude targets we calculated
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
            attitude_target.roll,
            attitude_target.pitch,
            attitude_target.yaw_rate
        );

        // Use altitude hold with velocity control
        float target_climb_rate_cms = -reference_state.vel_d * 100.0f;  // Convert m/s to cm/s
        pos_control->set_pos_target_U_from_climb_rate_cms(target_climb_rate_cms);
        pos_control->update_U_controller();
    } else {
        // Emergency fallback: level attitude with hover throttle
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);
        attitude_control->set_throttle_out(0.5f, true, g.throttle_filt);
    }
}

// Apply motor commands directly (legacy function, not used in state feedback mode)
// State feedback control now uses ArduPilot's attitude controller for proper integration
void ModeSmartPhoto99::apply_motor_commands(const Vector3f& moment_cmd, float thrust_cmd) {
    // This function is kept for backward compatibility but is not used
    // in the current state feedback implementation.
    // The state feedback controller now uses:
    //   attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad()
    // which properly integrates with ArduPilot's motor mixing and safety features.

    // If direct motor control is needed in the future, implement here:
    // 1. Convert moment_cmd to motor differential thrusts
    // 2. Use motors->output_armed_stabilizing() with custom mixing
    // 3. Handle motor safety limits and failsafes
}

// Update companion computer command
// Units: pos_ned [meters], vel_ned [m/s], yaw_target [radians], yaw_rate_target [rad/s]
void ModeSmartPhoto99::update_companion_command(const Vector3f& pos_ned, const Vector3f& vel_ned,
                                                 float yaw_target, float yaw_rate_target) {
    companion_cmd.position_ned = pos_ned;      // [meters] NED frame
    companion_cmd.velocity_ned = vel_ned;      // [m/s] NED frame
    companion_cmd.yaw = yaw_target;            // [radians] 0 = North
    companion_cmd.yaw_rate = yaw_rate_target;  // [rad/s]
    companion_cmd.timestamp_ms = AP_HAL::millis();  // [milliseconds]
    companion_cmd.valid = true;

    // Enable companion command mode on first valid command
    if (!smoothing.use_companion_cmd) {
        smoothing.use_companion_cmd = true;
        gcs().send_text(MAV_SEVERITY_INFO, "SMARTPHOTO: Companion control enabled");
    }
}

// Check if companion command is valid and recent
bool ModeSmartPhoto99::companion_command_valid() const {
    if (!companion_cmd.valid) {
        return false;
    }

    // Timeout after 500ms without updates
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - companion_cmd.timestamp_ms > 500) {
        return false;
    }

    return true;
}

// Calculate desired attitude from velocity command
// Uses the relationship: for small angles in NED frame
//   desired_pitch ≈ desired_accel_north / g
//   desired_roll ≈ -desired_accel_east / g
void ModeSmartPhoto99::calculate_desired_attitude_from_velocity(const Vector3f& vel_cmd,
                                                                  float& roll_target, float& pitch_target) {
    // Get current velocity from EKF
    Vector3f vel_current;
    if (!copter.ahrs.get_velocity_NED(vel_current)) {
        vel_current.zero();
    }

    // Calculate velocity errors
    Vector3f vel_error = vel_cmd - vel_current;

    // Velocity feedback gains (convert velocity error to acceleration command)
    const float vel_gain_xy = 2.0f;  // Proportional gain for velocity tracking

    // Desired horizontal accelerations (m/s^2)
    float accel_north = vel_error.x * vel_gain_xy;
    float accel_east = vel_error.y * vel_gain_xy;

    // Limit acceleration commands
    const float max_accel_xy = 3.0f;  // m/s^2
    accel_north = constrain_float(accel_north, -max_accel_xy, max_accel_xy);
    accel_east = constrain_float(accel_east, -max_accel_xy, max_accel_xy);

    // Convert accelerations to attitude targets
    const float gravity = 9.81f;  // m/s^2
    pitch_target = atanf(accel_north / gravity);
    roll_target = -atanf(accel_east / gravity);

    // Apply safety limits on attitude
    const float max_tilt_angle = radians(30.0f);  // 30 degrees max tilt
    pitch_target = constrain_float(pitch_target, -max_tilt_angle, max_tilt_angle);
    roll_target = constrain_float(roll_target, -max_tilt_angle, max_tilt_angle);
}

// Smooth attitude targets using slew rate limiter and low-pass filter
void ModeSmartPhoto99::smooth_attitude_targets(float roll_desired, float pitch_desired, float yaw_desired,
                                                float dt) {
    if (dt <= 0.0f || dt > 1.0f) {
        return;  // Invalid dt
    }

    // First-order low-pass filter coefficient
    // alpha = dt / (tc + dt), where tc is time constant
    float alpha = dt / (smoothing.attitude_tc + dt);

    // Calculate raw changes
    float roll_change = roll_desired - attitude_target.roll;
    float pitch_change = pitch_desired - attitude_target.pitch;
    float yaw_change = wrap_PI(yaw_desired - attitude_target.yaw);

    // Apply slew rate limiter
    float max_tilt_change = smoothing.max_tilt_rate * dt;
    roll_change = constrain_float(roll_change, -max_tilt_change, max_tilt_change);
    pitch_change = constrain_float(pitch_change, -max_tilt_change, max_tilt_change);

    float max_yaw_change = smoothing.max_yaw_rate * dt;
    yaw_change = constrain_float(yaw_change, -max_yaw_change, max_yaw_change);

    // Store previous values for rate calculation
    attitude_target.roll_prev = attitude_target.roll;
    attitude_target.pitch_prev = attitude_target.pitch;
    attitude_target.yaw_prev = attitude_target.yaw;

    // Apply low-pass filter to the rate-limited changes
    attitude_target.roll += roll_change * alpha;
    attitude_target.pitch += pitch_change * alpha;
    attitude_target.yaw = wrap_PI(attitude_target.yaw + yaw_change * alpha);

    // Update timestamp
    attitude_target.last_update_ms = AP_HAL::millis();
}

// Calculate attitude rates from attitude changes
void ModeSmartPhoto99::calculate_attitude_rates(float dt) {
    if (dt <= 0.0f || dt > 1.0f) {
        // Invalid dt, keep previous rates
        return;
    }

    // Calculate rates as finite differences
    float roll_rate_raw = (attitude_target.roll - attitude_target.roll_prev) / dt;
    float pitch_rate_raw = (attitude_target.pitch - attitude_target.pitch_prev) / dt;
    float yaw_rate_raw = wrap_PI(attitude_target.yaw - attitude_target.yaw_prev) / dt;

    // Apply low-pass filter to rates for smoothness
    const float rate_alpha = 0.3f;  // Filter coefficient for rate smoothing
    attitude_target.roll_rate = attitude_target.roll_rate * (1.0f - rate_alpha) + roll_rate_raw * rate_alpha;
    attitude_target.pitch_rate = attitude_target.pitch_rate * (1.0f - rate_alpha) + pitch_rate_raw * rate_alpha;
    attitude_target.yaw_rate = attitude_target.yaw_rate * (1.0f - rate_alpha) + yaw_rate_raw * rate_alpha;

    // Constrain rates to reasonable limits
    attitude_target.roll_rate = constrain_float(attitude_target.roll_rate, -smoothing.max_tilt_rate, smoothing.max_tilt_rate);
    attitude_target.pitch_rate = constrain_float(attitude_target.pitch_rate, -smoothing.max_tilt_rate, smoothing.max_tilt_rate);
    attitude_target.yaw_rate = constrain_float(attitude_target.yaw_rate, -smoothing.max_yaw_rate, smoothing.max_yaw_rate);
}

#endif  // MODE_SMARTPHOTO_ENABLED
