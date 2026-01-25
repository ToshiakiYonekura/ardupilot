#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    AP_Int8 get_int8Param() const { return _int8; }
    AP_Int16 get_int16Param() const { return _int16; }
    AP_Float get_floatParam() const { return _float; }

    // Mode 99 Autonomous Flight Mission Ready Parameter
    // Signals mission configuration completion to Raspberry Pi
    // 0 = Mission not configured, 1 = Mission configured and ready
    AP_Int8 get_mission_ready() const { return _mission_ready; }

private:
    // Put your parameter variable definitions here
    AP_Int8 _int8;
    AP_Int16 _int16;
    AP_Float _float;

    // Mode 99 Autonomous Flight Mission Ready Parameter
    AP_Int8 _mission_ready;
};
