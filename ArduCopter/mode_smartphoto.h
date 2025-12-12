// ArduCopter/mode_smartphoto.h
// System Identification Mode (Mode 98)

#pragma once

#include "mode.h"
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>

class ModeSmartPhoto : public Mode {
public:
    ModeSmartPhoto();

    // Required overrides
    Number mode_number() const override { return Number(98); }
    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:
    const char *name() const override { return "SYSID"; }
    const char *name4() const override { return "SID"; }

private:
    // State machine for system identification
    enum class State {
        INIT,
        TAKEOFF,
        CLIMB_TO_ALT,
        HOVER_STABILIZE,
        SYSID_ROLL,
        SYSID_PITCH,
        SYSID_YAW,
        SYSID_THROTTLE,
        COMPLETE,
        RTL
    };

    State state;
    uint32_t state_start_ms;

    // Target altitude for system identification (20m = 2000cm)
    float target_alt_cm;

    // System identification parameters
    struct SysidData {
        // Copter physical model (EDU650)
        float mass;                    // kg
        float Ixx, Iyy, Izz;          // Inertia moments

        // Motor configuration
        Vector3f motor_positions[4];   // Motor positions
        float motor_kv;                // KV value
        float max_thrust_per_motor;    // Max thrust per motor

        // Identification results
        float roll_rate_gain;
        float pitch_rate_gain;
        float yaw_rate_gain;
        float throttle_hover;

        // Data collection
        uint32_t sample_count;
        bool identification_complete;
    } sysid_data;

    // Home location for RTL
    Location home_loc;

    // Helper functions
    void load_EDU650_parameters();
    void execute_sysid_maneuver();
    void update_copter_parameters();
    bool check_altitude_reached();
    void transition_to_state(State new_state);

    // Parameter file I/O
    void save_identified_parameters();
    bool load_identified_parameters();
};
