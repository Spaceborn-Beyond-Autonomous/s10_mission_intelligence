#ifndef S10_MISSION_H
#define S10_MISSION_H

#include <stdint.h>
#include "s10_graph.h"
#include "../src/intent/mission_builder.h"

typedef enum
{
    S10_MISSION_STATE_IDLE = 0,
    S10_MISSION_STATE_PLANNING,
    S10_MISSION_STATE_EXECUTING,
    S10_MISSION_STATE_REPLANNING,
    S10_MISSION_STATE_COMPLETED,
    S10_MISSION_STATE_FAILED
} S10MissionState;

typedef struct
{
    uint32_t mission_id;
    S10MissionState state;
    MissionGraph graph;
    int current_node_id;
} S10Mission;

typedef struct
{
    S10Mission mission;
} S10MissionContext;

void s10_mission_state_set(
    S10Mission *mission,
    S10MissionState state
);

S10MissionState s10_mission_state_get(
    const S10Mission *mission
);

void s10_mission_context_init(
    S10MissionContext *context,
    uint32_t mission_id
);

S10Mission *s10_mission_context_get_mission(
    S10MissionContext *context
);

S10Mission *s10_mission_create(uint32_t mission_id);

void s10_mission_reset(S10Mission *mission);

S10ApplyStatus s10_mission_apply_change(
    S10Mission *mission,
    const S10ChangeEvent *event
);

#endif
