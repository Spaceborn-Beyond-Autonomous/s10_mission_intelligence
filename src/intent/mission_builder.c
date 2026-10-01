#include "mission_builder.h"
#include "graph_validator.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

MissionGraph build_mission(Command *commands, int count)
{
    MissionGraph graph = {0};

    if (commands == NULL || count <= 0 || count > S10_MAX_NODES) {
        return graph; // nothing to build, stays all-zero/NULL
    }

    graph.nodes = malloc((size_t)count * sizeof(MissionNode));
    if (graph.nodes == NULL) {
        return (MissionGraph){0};
    }
    graph.node_count = count;
    for (int i = 0; i < count; i++) {
        graph.nodes[i].id = i;
        graph.nodes[i].cmd = commands[i];   // copy the full command
    }

    int edge_count = count - 1; // sequential chain: node[i] -> node[i+1]
    if (edge_count > 0) {
        graph.edges = malloc((size_t)edge_count * sizeof(MissionEdge));
        if (graph.edges == NULL) {
            free(graph.nodes);
            return (MissionGraph){0};
        }
        graph.edge_count = edge_count;
        for (int i = 0; i < edge_count; i++) {
            graph.edges[i].from_id = i;
            graph.edges[i].to_id = i + 1;
        }
    }
    // edge_count == 0 for single-node missions: edges stays NULL, edge_count stays 0

    return graph;
}

void s10_graph_free(MissionGraph *g)
{
    if (g == NULL) {
        return;
    }
    free(g->nodes);
    free(g->edges);
    g->nodes = NULL;
    g->edges = NULL;
    g->node_count = 0;
    g->edge_count = 0;
}

// ---------- adapt to changed conditions ----------

#define S10_HOLD_S 30

// Case-insensitive string equality.
static int same(const char *a, const char *b)
{
    for (; *a && *b; a++, b++)
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return 0;
    return *a == *b;
}

// True if the command's position, area or route matches `area` (case-insensitive, whole
// string). An empty `area` never matches.
static int touches_area(const Command *c, const char *area)
{
    if (area[0] == '\0') return 0;
    return same(c->position, area) || same(c->area, area) || same(c->route_or_area, area);
}

// Returns a zero-initialised command of the given type (used for inserted HOVER/RTH/LAND nodes).
static Command simple(PrimitiveType t)
{
    Command c;
    memset(&c, 0, sizeof c);
    c.type = t;
    return c;
}

S10ApplyStatus s10_graph_apply_change(const MissionGraph *old, int current_node_id,
                                      const S10ChangeEvent *ev, MissionGraph *out)
{
    if (!out) return S10_APPLY_BAD_ARGS;
    memset(out, 0, sizeof *out);
    if (!old || !ev) return S10_APPLY_BAD_ARGS;
    if (s10_graph_validate(old).error != S10_GRAPH_OK) return S10_APPLY_BAD_INPUT_GRAPH;

    int n = old->node_count, cur = current_node_id;
    if (cur < 0 || cur >= n) return S10_APPLY_BAD_ARGS;

    Command *cmds = malloc((size_t)(n + 2) * sizeof(Command));
    if (!cmds) return S10_APPLY_NO_MEMORY;

    int to = n, hold = 0, append = -1, k = 0;

    if (ev->severity == S10_SEV_MEDIUM) {
        hold = 1;
    } else if (ev->severity == S10_SEV_HIGH) {
        for (int i = cur; i < n; i++) {
            if (i == ev->affected_node_id || touches_area(&old->nodes[i].cmd, ev->affected_area)) {
                to = i;
                append = RTH;
                break;
            }
        }
    } else if (ev->severity == S10_SEV_CRITICAL) {
        to = cur;
        append = (ev->type == S10_CHANGE_BATTERY) ? LAND_CMD : RTH;
    }

    for (int i = 0; i < cur; i++) cmds[k++] = old->nodes[i].cmd;      // history
    if (hold) { cmds[k] = simple(HOVER); cmds[k++].duration_s = S10_HOLD_S; }
    for (int i = cur; i < to; i++) cmds[k++] = old->nodes[i].cmd;     // surviving pending nodes
    if (append >= 0) cmds[k++] = simple((PrimitiveType)append);

    *out = build_mission(cmds, k);
    free(cmds);

    if (s10_graph_validate(out).error != S10_GRAPH_OK) {
        s10_graph_free(out);
        return S10_APPLY_INVALID_RESULT;
    }
    return (hold || append >= 0) ? S10_APPLY_CHANGED : S10_APPLY_UNCHANGED;
}