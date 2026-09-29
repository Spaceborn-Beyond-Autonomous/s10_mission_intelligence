#include "s10_mission.h"

void s10_mission_state_set(S10Mission *mission, S10MissionState state)
{
    if (mission == 0)
    {
        return;
    }

    mission->state = state;
}

S10MissionState s10_mission_state_get(const S10Mission *mission)
{
    if (mission == 0)
    {
        return S10_MISSION_STATE_FAILED;
    }

    return mission->state;
}
