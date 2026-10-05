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

S10ApplyStatus s10_mission_apply_change(
    S10Mission *mission,
    const S10ChangeEvent *event
)
{
    if (mission == 0 || event == 0)
    {
        return S10_APPLY_BAD_ARGS;
    }

    MissionGraph updated = {0};

    S10ApplyStatus status = s10_graph_apply_change(
        &mission->graph,
        mission->current_node_id,
        event,
        &updated
    );

    if (status == S10_APPLY_CHANGED)
    {
        s10_graph_free(&mission->graph);
        mission->graph = updated;
        mission->state = S10_MISSION_STATE_REPLANNING;
    }
    else if (status == S10_APPLY_UNCHANGED)
    {
        s10_graph_free(&updated);
    }

    return status;
}
