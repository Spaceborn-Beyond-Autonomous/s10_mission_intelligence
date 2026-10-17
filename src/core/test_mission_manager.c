#include "s10_mission.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(void)
{
    int count = 0;
    Command *commands = interpret_intent(
        parse_intent("deliver from A to B"), &count
    );

    if (commands == NULL || count <= 0) {
        printf("FAIL: Could not build test mission\n");
        free(commands);
        return 1;
    }

    S10Mission *mission = s10_mission_create(1);
    mission->graph = build_mission(commands, count);
    free(commands);

    if (mission->graph.nodes == NULL) {
        printf("FAIL: Could not build mission graph\n");
        return 1;
    }

    mission->state = S10_MISSION_STATE_EXECUTING;
    mission->current_node_id = 2;

    S10ChangeEvent event = {
        S10_CHANGE_WEATHER, S10_SEV_MEDIUM, -1, ""
    };

    S10ApplyStatus status = s10_mission_apply_change(mission, &event);

    int passed =
        status == S10_APPLY_CHANGED &&
        mission->state == S10_MISSION_STATE_REPLANNING &&
        mission->graph.node_count == count + 1;

    printf("Mission manager integration: %s\n",
           passed ? "PASS" : "FAIL");
    printf("status=%d state=%d nodes=%d\n",
           status, mission->state, mission->graph.node_count);

    s10_mission_reset(mission);
    return passed ? 0 : 1;
}
