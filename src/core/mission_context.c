#include "s10_mission.h"

void s10_mission_context_init(
    S10MissionContext *context,
    uint32_t mission_id
)
{
    if (context == 0)
    {
        return;
    }

    context->mission.mission_id = mission_id;
    context->mission.state = S10_MISSION_STATE_IDLE;
    context->mission.graph = (MissionGraph){0};
    context->mission.current_node_id = 0;
}

S10Mission 
*s10_mission_context_get_mission(
    S10MissionContext *context
)
{
    if (context == 0)
    {
        return 0;
    }

    return &context->mission;
}
