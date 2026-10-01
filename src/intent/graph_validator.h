#ifndef S10_GRAPH_VALIDATOR_H
#define S10_GRAPH_VALIDATOR_H

#include "../../include/s10_graph.h"

typedef enum {
    S10_GRAPH_OK = 0,
    S10_GRAPH_EMPTY,
    S10_GRAPH_TOO_LARGE,
    S10_GRAPH_DUPLICATE_ID,
    S10_GRAPH_BAD_ID,        // id != its index in the nodes array
    S10_GRAPH_BAD_EDGE,      // edge endpoint is not a real node id
    S10_GRAPH_NOT_A_CHAIN,   // wrong edge count, branch, merge, or no single start
    S10_GRAPH_UNREACHABLE,   // node not reachable from the start (also catches loops)
    S10_GRAPH_NO_TERMINAL,   // chain does not end in RTH or LAND_CMD
    S10_GRAPH_MISSING_PARAM  // required field missing for the node type
} S10GraphError;

typedef struct {
    S10GraphError error;
    int node_id;             // offending node/endpoint id, or -1 if not applicable
} S10GraphResult;

// Structure-only validation.
// Checks run in the order listed above and stop at the first failure.
S10GraphResult s10_graph_validate(const MissionGraph *g);

const char *s10_graph_error_str(S10GraphError e);

#endif