#include "UserParameters.h"
#include "config.h"

#if USER_PARAMS_ENABLED
// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_INT8", 0, UserParameters, _int8, 0),
    AP_GROUPINFO("_INT16", 1, UserParameters, _int16, 0),
    AP_GROUPINFO("_FLOAT", 2, UserParameters, _float, 0),

    // @Param: _MISSION_RDY
    // @DisplayName: Mission Ready Flag for Mode 99
    // @Description: Signals mission configuration completion to Raspberry Pi companion computer. Set to 1 when mission is configured in Mission Planner. Raspberry Pi monitors this parameter to trigger autonomous flight sequence.
    // @Values: 0:Not Ready, 1:Mission Configured
    // @User: Advanced
    AP_GROUPINFO("_MISSION_RDY", 3, UserParameters, _mission_ready, 0),

    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
