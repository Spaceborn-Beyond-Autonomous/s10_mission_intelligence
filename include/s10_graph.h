#ifndef S10_GRAPH_H
#define S10_GRAPH_H

#include "../src/intent/command_interpreter.h"  // for Command, PrimitiveType

typedef struct {
    int id;          // unique within this graph; equals the command's index (0, 1, 2...)
    Command cmd;     // full task data (type, position, duration_s, area, payload, ...)
} MissionNode;

typedef struct {
    int from_id;     // id of the MissionNode this edge starts at
    int to_id;       // id of the MissionNode this edge ends at
} MissionEdge;

typedef struct {
    MissionNode *nodes;
    int node_count;
    MissionEdge *edges;
    int edge_count;
    // MVP: strict chain, node[i] -> node[i+1], so edge_count == node_count - 1
} MissionGraph;

// Frees nodes/edges and zeroes the struct. Safe on an empty or already-freed graph.
void s10_graph_free(MissionGraph *g);

#endif