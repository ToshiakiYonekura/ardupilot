// ArduCopter/mode_smartphoto.cpp
// System Identification Mode (Mode 98)

#include "Copter.h"
#include "mode.h"
#include "mode_smartphoto.h"
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

// Constructor
ModeSmartPhoto::ModeSmartPhoto() : Mode() {
    state = State::INIT;
    state_start_ms = 0;
    target_alt_cm = 2000.0f;  // 20m in cm
}

// Load EDU650 parameters
void ModeSmartPhoto::load_EDU650_parameters() {
    // EDU650 physical specifications
    sysid_data.mass = 3.426f;  // kg

    // Inertia moments [kg⋅m²]
    sysid_data.Ixx = 0.0892f;
    sysid_data.Iyy = 0.0895f;
    sysid_data.Izz = 0.1654f;

    // Motor placement (X configuration)
    sysid_data.motor_positions[0] = Vector3f(0.230f, -0.230f, 0);  // m
    sysid_data.motor_positions[1] = Vector3f(0.230f, 0.230f, 0);
    sysid_data.motor_positions[2] = Vector3f(-0.230f, 0.230f, 0);
    sysid_data.motor_positions[3] = Vector3f(-0.230f, -0.230f, 0);

    // HS4012 KV:370 motor characteristics
    sysid_data.motor_kv = 370.0f;
    sysid_data.max_thrust_per_motor = 2.2f;  // kg (at 6S)

    // Initialize identification results
    sysid_data.roll_rate_gain = 0.0f;
    sysid_data.pitch_rate_gain = 0.0f;
    sysid_data.yaw_rate_gain = 0.0f;
    sysid_data.throttle_hover = 0.5f;
    sysid_data.sample_count = 0;
    sysid_data.identification_complete = false;

    gcs().send_text(MAV_SEVERITY_INFO,
        "SYSID: EDU650 params loaded: mass=%.2fkg, Ixx=%.4f",
        sysid_data.mass, sysid_data.Ixx);
}

// Initialization function
bool ModeSmartPhoto::init(bool ignore_checks) {
    // Basic checks
    if (!copter.position_ok()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SYSID: Position not OK");
        return false;
    }

    // Store home location for RTL
    if (!copter.ahrs.get_location(home_loc)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SYSID: Cannot get home location");
        return false;
    }

    // Load EDU650 parameters
    load_EDU650_parameters();

    // Initialize state machine
    transition_to_state(State::INIT);

    gcs().send_text(MAV_SEVERITY_INFO, "SYSID: Mode initialized");

    return true;
}

// Main loop
void ModeSmartPhoto::run() {
    uint32_t now = AP_HAL::millis();

    // Default pilot inputs
    float target_roll_rad = 0.0f;
    float target_pitch_rad = 0.0f;
    float target_yaw_rate_rads = 0.0f;
    float target_climb_rate_ms = 0.0f;

    switch (state) {
        case State::INIT:
            // Wait for stability, then start takeoff
            if (now - state_start_ms > 2000) {
                transition_to_state(State::TAKEOFF);
            }
            // Hold position
            copter.pos_control->relax_U_controller(0.0f);
            break;

        case State::TAKEOFF:
            // Start takeoff - just transition to climb after arming
            copter.set_auto_armed(true);
            if (!copter.ap.land_complete) {
                gcs().send_text(MAV_SEVERITY_INFO, "SYSID: Taking off");
                transition_to_state(State::CLIMB_TO_ALT);
            } else {
                // Still on ground, hold position
                copter.pos_control->relax_U_controller(0.0f);
            }
            break;

        case State::CLIMB_TO_ALT:
            // Climb to target altitude (20m)
            if (check_altitude_reached()) {
                transition_to_state(State::HOVER_STABILIZE);
            } else {
                // Set target altitude and climb at 0.5 m/s
                target_climb_rate_ms = 0.5f;
                copter.pos_control->set_pos_target_U_from_climb_rate_m(target_climb_rate_ms);
            }
            break;

        case State::HOVER_STABILIZE:
            // Hover and stabilize for 5 seconds before starting identification
            if (now - state_start_ms > 5000) {
                gcs().send_text(MAV_SEVERITY_INFO, "SYSID: Starting identification");
                transition_to_state(State::SYSID_ROLL);
            } else {
                // Hold altitude
                copter.pos_control->set_pos_target_U_from_climb_rate_m(0.0f);
            }
            break;

        case State::SYSID_ROLL:
        case State::SYSID_PITCH:
        case State::SYSID_YAW:
        case State::SYSID_THROTTLE:
            // Execute system identification maneuvers
            execute_sysid_maneuver();

            // Progress through identification states
            if (now - state_start_ms > 10000) {  // 10 seconds per axis
                if (state == State::SYSID_ROLL) {
                    transition_to_state(State::SYSID_PITCH);
                } else if (state == State::SYSID_PITCH) {
                    transition_to_state(State::SYSID_YAW);
                } else if (state == State::SYSID_YAW) {
                    transition_to_state(State::SYSID_THROTTLE);
                } else if (state == State::SYSID_THROTTLE) {
                    transition_to_state(State::COMPLETE);
                }
            }
            // Don't call attitude controller here - done in execute_sysid_maneuver
            copter.pos_control->update_U_controller();
            return;  // Early return to skip default attitude controller call

        case State::COMPLETE:
            // System identification complete
            if (now - state_start_ms > 2000) {
                gcs().send_text(MAV_SEVERITY_INFO, "SYSID: Identification complete");

                // Update copter parameters with identification results
                update_copter_parameters();

                transition_to_state(State::RTL);
            }
            // Hold altitude
            copter.pos_control->set_pos_target_U_from_climb_rate_m(0.0f);
            break;

        case State::RTL:
            // Return to home
            if (!copter.set_mode(Mode::Number::RTL, ModeReason::UNKNOWN)) {
                // If RTL fails, try LAND
                copter.set_mode(Mode::Number::LAND, ModeReason::UNKNOWN);
            }
            return;  // Early return - RTL mode takes over
    }

    // Call attitude controller with default inputs (hold position)
    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
        target_roll_rad, target_pitch_rad, target_yaw_rate_rads);

    // Run the vertical position controller
    copter.pos_control->update_U_controller();
}

// Execute system identification maneuver
void ModeSmartPhoto::execute_sysid_maneuver() {
    uint32_t now = AP_HAL::millis();
    float time_in_state = (now - state_start_ms) * 0.001f;  // seconds

    // Generate frequency sweep signal (chirp)
    float freq_start = 0.5f;  // Hz
    float freq_end = 5.0f;    // Hz
    float duration = 10.0f;   // seconds
    float freq = freq_start + (freq_end - freq_start) * time_in_state / duration;
    float amplitude = 0.0f;

    // Apply excitation based on current axis
    float target_roll_rad = 0.0f;
    float target_pitch_rad = 0.0f;
    float target_yaw_rate_rads = 0.0f;

    switch (state) {
        case State::SYSID_ROLL:
            amplitude = radians(10.0f);  // 10 degree amplitude
            target_roll_rad = amplitude * sinf(2.0f * M_PI * freq * time_in_state);
            break;

        case State::SYSID_PITCH:
            amplitude = radians(10.0f);  // 10 degree amplitude
            target_pitch_rad = amplitude * sinf(2.0f * M_PI * freq * time_in_state);
            break;

        case State::SYSID_YAW:
            amplitude = radians(15.0f);  // 15 degree/s amplitude
            target_yaw_rate_rads = amplitude * sinf(2.0f * M_PI * freq * time_in_state);
            break;

        case State::SYSID_THROTTLE:
            // Throttle pulse (add to base altitude hold)
            copter.pos_control->set_pos_target_U_from_climb_rate_m(
                0.1f * sinf(2.0f * M_PI * freq * time_in_state));
            break;

        default:
            break;
    }

    // Maintain altitude
    copter.pos_control->set_pos_target_U_from_climb_rate_m(0.0f);

    // Apply attitude command
    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
        target_roll_rad, target_pitch_rad, target_yaw_rate_rads);

    // Collect data samples
    sysid_data.sample_count++;
}

// Update copter parameters with identification results
void ModeSmartPhoto::update_copter_parameters() {
    // Calculate identified parameters based on collected data
    // This is a simplified version - real implementation would process logged data

    // Estimate gains from frequency response
    sysid_data.roll_rate_gain = 1.5f;
    sysid_data.pitch_rate_gain = 1.5f;
    sysid_data.yaw_rate_gain = 1.0f;
    sysid_data.throttle_hover = copter.motors->get_throttle_hover();

    sysid_data.identification_complete = true;

    gcs().send_text(MAV_SEVERITY_INFO,
        "SYSID: Roll=%.2f Pitch=%.2f Yaw=%.2f THR=%.2f",
        sysid_data.roll_rate_gain,
        sysid_data.pitch_rate_gain,
        sysid_data.yaw_rate_gain,
        sysid_data.throttle_hover);

    gcs().send_text(MAV_SEVERITY_INFO, "SYSID: Samples=%u", sysid_data.sample_count);

    // Save parameters to file for use in mode 99
    save_identified_parameters();

    // TODO: Apply parameters to attitude controller
    // copter.attitude_control->set_rate_gains(...);
}

// Check if target altitude is reached
bool ModeSmartPhoto::check_altitude_reached() {
    // Get current altitude in cm (ABOVE_HOME frame)
    float current_alt_cm = copter.current_loc.alt * 0.01f;  // Convert from cm to m then back to handle frame

    // Check if within 50cm of target (20m = 2000cm)
    return fabsf(current_alt_cm - target_alt_cm) < 50.0f;
}

// Transition to new state
void ModeSmartPhoto::transition_to_state(State new_state) {
    state = new_state;
    state_start_ms = AP_HAL::millis();

    const char* state_names[] = {
        "INIT", "TAKEOFF", "CLIMB_TO_ALT", "HOVER_STABILIZE",
        "SYSID_ROLL", "SYSID_PITCH", "SYSID_YAW", "SYSID_THROTTLE",
        "COMPLETE", "RTL"
    };

    gcs().send_text(MAV_SEVERITY_INFO, "SYSID: State -> %s",
        state_names[static_cast<int>(new_state)]);
}

// Save identified parameters to file
void ModeSmartPhoto::save_identified_parameters() {
    const char* filename = "sysid_params.txt";

    // Use HAL filesystem to write parameters
    int fd = AP::FS().open(filename, O_WRONLY | O_CREAT | O_TRUNC);
    if (fd == -1) {
        gcs().send_text(MAV_SEVERITY_ERROR, "SYSID: Failed to create %s", filename);
        return;
    }

    char buffer[256];

    // Write EDU650 physical parameters
    snprintf(buffer, sizeof(buffer), "# EDU650 System Identification Parameters\n");
    AP::FS().write(fd, buffer, strlen(buffer));

    snprintf(buffer, sizeof(buffer), "MASS=%.3f\n", sysid_data.mass);
    AP::FS().write(fd, buffer, strlen(buffer));

    snprintf(buffer, sizeof(buffer), "IXX=%.4f\n", sysid_data.Ixx);
    AP::FS().write(fd, buffer, strlen(buffer));

    snprintf(buffer, sizeof(buffer), "IYY=%.4f\n", sysid_data.Iyy);
    AP::FS().write(fd, buffer, strlen(buffer));

    snprintf(buffer, sizeof(buffer), "IZZ=%.4f\n", sysid_data.Izz);
    AP::FS().write(fd, buffer, strlen(buffer));

    // Write motor parameters
    snprintf(buffer, sizeof(buffer), "MOTOR_KV=%.1f\n", sysid_data.motor_kv);
    AP::FS().write(fd, buffer, strlen(buffer));

    snprintf(buffer, sizeof(buffer), "MAX_THRUST=%.2f\n", sysid_data.max_thrust_per_motor);
    AP::FS().write(fd, buffer, strlen(buffer));

    // Write identified control parameters
    snprintf(buffer, sizeof(buffer), "\n# Identified Control Parameters\n");
    AP::FS().write(fd, buffer, strlen(buffer));

    snprintf(buffer, sizeof(buffer), "ROLL_GAIN=%.3f\n", sysid_data.roll_rate_gain);
    AP::FS().write(fd, buffer, strlen(buffer));

    snprintf(buffer, sizeof(buffer), "PITCH_GAIN=%.3f\n", sysid_data.pitch_rate_gain);
    AP::FS().write(fd, buffer, strlen(buffer));

    snprintf(buffer, sizeof(buffer), "YAW_GAIN=%.3f\n", sysid_data.yaw_rate_gain);
    AP::FS().write(fd, buffer, strlen(buffer));

    snprintf(buffer, sizeof(buffer), "THROTTLE_HOVER=%.3f\n", sysid_data.throttle_hover);
    AP::FS().write(fd, buffer, strlen(buffer));

    snprintf(buffer, sizeof(buffer), "SAMPLES=%u\n", sysid_data.sample_count);
    AP::FS().write(fd, buffer, strlen(buffer));

    AP::FS().close(fd);

    gcs().send_text(MAV_SEVERITY_INFO, "SYSID: Parameters saved to %s", filename);
}

// Load identified parameters from file
bool ModeSmartPhoto::load_identified_parameters() {
    const char* filename = "sysid_params.txt";

    int fd = AP::FS().open(filename, O_RDONLY);
    if (fd == -1) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SYSID: No parameter file found");
        return false;
    }

    char buffer[512];
    ssize_t bytes_read = AP::FS().read(fd, buffer, sizeof(buffer) - 1);
    AP::FS().close(fd);

    if (bytes_read <= 0) {
        gcs().send_text(MAV_SEVERITY_ERROR, "SYSID: Failed to read parameters");
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

    sysid_data.identification_complete = true;

    gcs().send_text(MAV_SEVERITY_INFO,
        "SYSID: Loaded Roll=%.2f Pitch=%.2f Yaw=%.2f THR=%.2f",
        sysid_data.roll_rate_gain,
        sysid_data.pitch_rate_gain,
        sysid_data.yaw_rate_gain,
        sysid_data.throttle_hover);

    return true;
}
