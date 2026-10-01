#include "command_interpreter.h"
#include <stdlib.h>
#include <string.h>

static Command make_command(PrimitiveType type) {
    Command c = {0};
    c.type = type;
    return c;
}

static void set_str(char *dst, size_t dst_size, const char *src) {
    strncpy(dst, src, dst_size - 1);
    dst[dst_size - 1] = '\0';
}

Command *interpret_intent(Intent intent, int *out_count)
{
    switch (intent.type) {

    case SURVEY: {
        // [GOTO(area), SCAN(area), RTH]
        Command *cmds = malloc(3 * sizeof(Command));
        if (!cmds) { *out_count = 0; return NULL; }
        cmds[0] = make_command(GOTO);
        set_str(cmds[0].position, sizeof(cmds[0].position), intent.area);
        cmds[1] = make_command(SCAN);
        set_str(cmds[1].area, sizeof(cmds[1].area), intent.area);
        cmds[2] = make_command(RTH);
        *out_count = 3;
        return cmds;
    }

    case DELIVER: {
        // [GOTO(pickup), HOVER, PICKUP, GOTO(dropoff), HOVER, DROPOFF, RTH]
        Command *cmds = malloc(7 * sizeof(Command));
        if (!cmds) { *out_count = 0; return NULL; }
        cmds[0] = make_command(GOTO);
        set_str(cmds[0].position, sizeof(cmds[0].position), intent.pickup_location);
        cmds[1] = make_command(HOVER);
        cmds[1].duration_s = 5;
        cmds[2] = make_command(PICKUP);
        set_str(cmds[2].payload, sizeof(cmds[2].payload), intent.payload);
        cmds[3] = make_command(GOTO);
        set_str(cmds[3].position, sizeof(cmds[3].position), intent.dropoff_location);
        cmds[4] = make_command(HOVER);
        cmds[4].duration_s = 3;
        cmds[5] = make_command(DROPOFF);
        set_str(cmds[5].payload, sizeof(cmds[5].payload), intent.payload);
        cmds[6] = make_command(RTH);
        *out_count = 7;
        return cmds;
    }

    case PATROL: {
        // [PATROL_CMD(route_or_area, repetitions), RTH]
        // No leading GOTO here: unlike SURVEY's area, we don't have any
        // position-shaped value to hand it (route_or_area is a name/region
        // reference, not a single point) — resolving that into real
        // waypoints is entirely waypoint_generator's job downstream.
        Command *cmds = malloc(2 * sizeof(Command));
        if (!cmds) { *out_count = 0; return NULL; }
        cmds[0] = make_command(PATROL_CMD);
        set_str(cmds[0].route_or_area, sizeof(cmds[0].route_or_area), intent.route_or_area);
        cmds[0].repetitions = intent.repetitions;
        cmds[1] = make_command(RTH);
        *out_count = 2;
        return cmds;
    }

    case RETURN_TO_BASE: {
        // [RTH]
        Command *cmds = malloc(1 * sizeof(Command));
        if (!cmds) { *out_count = 0; return NULL; }
        cmds[0] = make_command(RTH);
        *out_count = 1;
        return cmds;
    }

    case LAND: {
        // [GOTO(location), LAND_CMD] if location given, else just [LAND_CMD]
        int has_location = intent.location[0] != '\0';
        int n = has_location ? 2 : 1;
        Command *cmds = malloc((size_t)n * sizeof(Command));
        if (!cmds) { *out_count = 0; return NULL; }
        int i = 0;
        if (has_location) {
            cmds[i] = make_command(GOTO);
            set_str(cmds[i].position, sizeof(cmds[i].position), intent.location);
            i++;
        }
        cmds[i] = make_command(LAND_CMD);
        *out_count = n;
        return cmds;
    }

    case UNKNOWN:
    default:
        *out_count = 0;
        return NULL;
    }
}