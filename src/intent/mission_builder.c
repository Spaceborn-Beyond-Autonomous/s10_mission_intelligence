#include "mission_builder.h"
#include <stdlib.h>

MissionGraph build_mission(Command *commands, int count)
{
    MissionGraph graph = {0};

    if (count <= 0) {
        return graph; // nothing to build — stays all-zero/NULL
    }

    graph.nodes = malloc((size_t)count * sizeof(MissionNode));
    graph.node_count = count;
    for (int i = 0; i < count; i++) {
        graph.nodes[i].id = i;
        graph.nodes[i].type = commands[i].type;
    }

    int edge_count = count - 1; // sequential chain: node[i] -> node[i+1]
    if (edge_count > 0) {
        graph.edges = malloc((size_t)edge_count * sizeof(MissionEdge));
        graph.edge_count = edge_count;
        for (int i = 0; i < edge_count; i++) {
            graph.edges[i].from_id = i;
            graph.edges[i].to_id = i + 1;
        }
    }
    // edge_count == 0 happens for RETURN_TO_BASE / plain LAND (single node,
    // no edges needed) — graph.edges stays NULL, graph.edge_count stays 0

    return graph;
}