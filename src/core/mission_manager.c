#include "s10_mission.h"

S10Mission *s10_mission_create(uint32_t mission_id)
{
    static S10Mission mission;

    mission.mission_id = mission_id;
    mission.state = S10_MISSION_STATE_IDLE;

    return &mission;
}

void s10_mission_reset(S10Mission *mission)
{
    if (mission == 0)
    {
        return;
    }

    mission->state = S10_MISSION_STATE_IDLE;
}
