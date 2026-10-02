#include "s10_mission.h"

S10Mission *s10_mission_create(uint32_t mission_id)
{
    static S10Mission mission;

    s10_graph_free(&mission.graph);

    mission.mission_id = mission_id;
    mission.state = S10_MISSION_STATE_IDLE;
    mission.current_node_id = 0;

    return &mission;
}

void s10_mission_reset(S10Mission *mission)
{
    if (mission == 0)
    {
        return;
    }

    s10_graph_free(&mission->graph);

    mission->state = S10_MISSION_STATE_IDLE;
    mission->current_node_id = 0;
}
