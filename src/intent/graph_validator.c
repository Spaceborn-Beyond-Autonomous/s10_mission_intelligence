#include "graph_validator.h"
#include <stddef.h>

static S10GraphResult fail(S10GraphError e, int id)
{
    S10GraphResult r = { e, id };
    return r;
}

S10GraphResult s10_graph_validate(const MissionGraph *g)
{
    if (g == NULL || g->nodes == NULL || g->node_count <= 0) return fail(S10_GRAPH_EMPTY, -1);
    int n = g->node_count;
    if (n > S10_MAX_NODES) return fail(S10_GRAPH_TOO_LARGE, -1);

    // ids: unique, and equal to the array index
    for (int i = 0; i < n; i++)
        for (int j = 0; j < i; j++)
            if (g->nodes[i].id == g->nodes[j].id) return fail(S10_GRAPH_DUPLICATE_ID, g->nodes[i].id);
    for (int i = 0; i < n; i++)
        if (g->nodes[i].id != i) return fail(S10_GRAPH_BAD_ID, g->nodes[i].id);

    // edges: valid endpoints
    if (g->edge_count < 0 || (g->edge_count > 0 && g->edges == NULL))
        return fail(S10_GRAPH_NOT_A_CHAIN, -1);
    for (int e = 0; e < g->edge_count; e++) {
        int f = g->edges[e].from_id, t = g->edges[e].to_id;
        if (f < 0 || f >= n) return fail(S10_GRAPH_BAD_EDGE, f);
        if (t < 0 || t >= n) return fail(S10_GRAPH_BAD_EDGE, t);
    }

    // chain shape: n-1 edges, at most one edge in and out of every node, one start
    if (g->edge_count != n - 1) return fail(S10_GRAPH_NOT_A_CHAIN, -1);

    int in_deg[S10_MAX_NODES] = {0}, out_deg[S10_MAX_NODES] = {0}, next[S10_MAX_NODES];
    for (int i = 0; i < n; i++) next[i] = -1;
    for (int e = 0; e < g->edge_count; e++) {
        int f = g->edges[e].from_id, t = g->edges[e].to_id;
        out_deg[f]++;
        in_deg[t]++;
        next[f] = t;
    }
    int start = -1, starts = 0;
    for (int i = 0; i < n; i++) {
        if (out_deg[i] > 1 || in_deg[i] > 1) return fail(S10_GRAPH_NOT_A_CHAIN, i);
        if (in_deg[i] == 0) { start = i; starts++; }
    }
    if (starts != 1) return fail(S10_GRAPH_NOT_A_CHAIN, -1);

    // reachability: walk the chain from the start
    int visited[S10_MAX_NODES] = {0};
    int count = 0, last = start, cur = start;
    while (cur != -1 && !visited[cur]) {
        visited[cur] = 1;
        last = cur;
        count++;
        cur = next[cur];
    }
    if (count < n) {
        for (int i = 0; i < n; i++)
            if (!visited[i]) return fail(S10_GRAPH_UNREACHABLE, i);
    }

    // terminal: the end of the chain must be RTH or LAND_CMD
    PrimitiveType lt = g->nodes[last].cmd.type;
    if (lt != RTH && lt != LAND_CMD) return fail(S10_GRAPH_NO_TERMINAL, last);

    // required parameters per node type
    for (int i = 0; i < n; i++) {
        const Command *c = &g->nodes[i].cmd;
        switch (c->type) {
            case GOTO:       if (c->position[0] == '\0')      return fail(S10_GRAPH_MISSING_PARAM, i); break;
            case HOVER:      if (c->duration_s <= 0)          return fail(S10_GRAPH_MISSING_PARAM, i); break;
            case SCAN:       if (c->area[0] == '\0')          return fail(S10_GRAPH_MISSING_PARAM, i); break;
            case PATROL_CMD:
                if (c->route_or_area[0] == '\0' || c->repetitions != 1) {
                    return fail(S10_GRAPH_MISSING_PARAM, i);
                }
                break;
            default: break;  // PICKUP/DROPOFF payload is optional; RTH, LAND_CMD, TAKEOFF_CMD need nothing
        }
    }

    return fail(S10_GRAPH_OK, -1);
}

const char *s10_graph_error_str(S10GraphError e)
{
    switch (e) {
        case S10_GRAPH_OK:            return "OK";
        case S10_GRAPH_EMPTY:         return "EMPTY";
        case S10_GRAPH_TOO_LARGE:     return "TOO_LARGE";
        case S10_GRAPH_DUPLICATE_ID:  return "DUPLICATE_ID";
        case S10_GRAPH_BAD_ID:        return "BAD_ID";
        case S10_GRAPH_BAD_EDGE:      return "BAD_EDGE";
        case S10_GRAPH_NOT_A_CHAIN:   return "NOT_A_CHAIN";
        case S10_GRAPH_UNREACHABLE:   return "UNREACHABLE";
        case S10_GRAPH_NO_TERMINAL:   return "NO_TERMINAL";
        case S10_GRAPH_MISSING_PARAM: return "MISSING_PARAM";
    }
    return "UNKNOWN";
}
