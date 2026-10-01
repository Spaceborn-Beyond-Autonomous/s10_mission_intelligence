#include "intent_parser.h"
#include "command_interpreter.h"
#include "mission_builder.h"
#include <stdio.h>
#include <stdlib.h>

static const char *prim_name(PrimitiveType t) {
    switch (t) {
        case GOTO: return "GOTO";
        case HOVER: return "HOVER";
        case SCAN: return "SCAN";
        case PICKUP: return "PICKUP";
        case DROPOFF: return "DROPOFF";
        case LAND_CMD: return "LAND_CMD";
        case RTH: return "RTH";
        case PATROL_CMD: return "PATROL_CMD";
    }
    return "???";
}

static void run(const char *text) {
    Intent intent = parse_intent(text);
    int count;
    Command *cmds = interpret_intent(intent, &count);
    MissionGraph graph = build_mission(cmds, count);

    printf("INPUT: \"%s\"\n", text);
    printf("  nodes=%d edges=%d\n", graph.node_count, graph.edge_count);
    for (int i = 0; i < graph.node_count; i++) {
        printf("    node[%d] id=%d type=%s\n", i, graph.nodes[i].id, prim_name(graph.nodes[i].type));
    }
    for (int i = 0; i < graph.edge_count; i++) {
        printf("    edge[%d] %d -> %d\n", i, graph.edges[i].from_id, graph.edges[i].to_id);
    }
    printf("\n");

    free(cmds);
    free(graph.nodes);
    free(graph.edges);
}

int main(void) {
    run("Survey North Field");
    run("deliver from Toronto to Berlin carrying medical supplies");
    run("please patrol Zone Alpha");
    run("return to base");
    run("land");
    run("fly around randomly");
    return 0;
}