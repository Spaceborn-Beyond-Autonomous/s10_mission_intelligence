#ifndef S10_GRAPH_H
#define S10_GRAPH_H

#include "../src/intent/command_interpreter.h"  // for PrimitiveType

typedef struct {
    int id;                  // unique within this graph; for MVP, the command's index (0, 1, 2...)
    PrimitiveType type;      // which primitive task this node represents (GOTO, SCAN, RTH, ...)
    
} MissionNode;

typedef struct {
    int from_id;              // id of the MissionNode this edge starts at
    int to_id;                // id of the MissionNode this edge ends at
    
} MissionEdge;

typedef struct {
    MissionNode *nodes;
    int node_count;
    MissionEdge *edges;
    int edge_count;
    // MVP: mission_builder connects nodes in strict recipe order
    // (node[i] -> node[i+1]), so edge_count == node_count - 1
} MissionGraph;

#endif