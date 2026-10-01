#include "intent_parser.h"
#include "command_interpreter.h"
#include "mission_builder.h"
#include "graph_validator.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures = 0;

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
        case TAKEOFF_CMD: return "TAKEOFF_CMD";
    }
    return "???";
}

static void print_node(const MissionNode *n) {
    const Command *c = &n->cmd;
    printf("    node id=%d type=%s", n->id, prim_name(c->type));
    if (c->position[0])      printf(" position=\"%s\"", c->position);
    if (c->area[0])          printf(" area=\"%s\"", c->area);
    if (c->payload[0])       printf(" payload=\"%s\"", c->payload);
    if (c->route_or_area[0]) printf(" route=\"%s\"", c->route_or_area);
    if (c->duration_s)       printf(" duration_s=%d", c->duration_s);
    if (c->repetitions)      printf(" reps=%d", c->repetitions);
    printf("\n");
}

// parse -> interpret -> build -> validate. Checks node count, first/last type
// (-1 skips) and the validator result.
static void run(const char *text, int exp_nodes, int exp_first, int exp_last, S10GraphError exp_err) {
    Intent intent = parse_intent(text);
    int count = 0;
    Command *cmds = interpret_intent(intent, &count);
    MissionGraph graph = build_mission(cmds, count);
    S10GraphResult vr = s10_graph_validate(&graph);

    printf("INPUT: \"%s\"\n", text);
    printf("  nodes=%d edges=%d\n", graph.node_count, graph.edge_count);
    for (int i = 0; i < graph.node_count; i++) print_node(&graph.nodes[i]);
    for (int i = 0; i < graph.edge_count; i++)
        printf("    edge %d -> %d\n", graph.edges[i].from_id, graph.edges[i].to_id);
    printf("  validate: %s (node %d)\n", s10_graph_error_str(vr.error), vr.node_id);

    int ok = (graph.node_count == exp_nodes) && (vr.error == exp_err);
    if (ok && exp_nodes > 0) {
        ok = (graph.edge_count == exp_nodes - 1);
        if (ok && exp_first >= 0) ok = ((int)graph.nodes[0].cmd.type == exp_first);
        if (ok && exp_last >= 0)  ok = ((int)graph.nodes[exp_nodes - 1].cmd.type == exp_last);
    }
    printf("  %s\n\n", ok ? "PASS" : "FAIL");
    if (!ok) failures++;

    free(cmds);
    s10_graph_free(&graph);
}

// ---------- hand-built graphs for validator error tests ----------

static MissionGraph make_chain(const PrimitiveType *types, int n) {
    Command *cmds = calloc((size_t)n, sizeof(Command));
    for (int i = 0; i < n; i++) {
        cmds[i].type = types[i];
        switch (types[i]) {
            case GOTO:       strcpy(cmds[i].position, "A"); break;
            case HOVER:      cmds[i].duration_s = 5; break;
            case SCAN:       strcpy(cmds[i].area, "A"); break;
            case PICKUP:
            case DROPOFF:    strcpy(cmds[i].payload, "p"); break;
            case PATROL_CMD: strcpy(cmds[i].route_or_area, "r"); cmds[i].repetitions = 1; break;
            default: break;
        }
    }
    MissionGraph g = build_mission(cmds, n);
    free(cmds);
    return g;
}

// exp_id of -2 means "don't check the node id"
static void check(const char *name, S10GraphResult r, S10GraphError exp_err, int exp_id) {
    int ok = (r.error == exp_err) && (exp_id == -2 || r.node_id == exp_id);
    printf("%-30s -> %-13s (node %2d)  %s\n", name, s10_graph_error_str(r.error), r.node_id,
           ok ? "PASS" : "FAIL");
    if (!ok) failures++;
}

static void test_validator_errors(void) {
    PrimitiveType t2[] = {GOTO, RTH};
    PrimitiveType t3[] = {GOTO, SCAN, RTH};
    PrimitiveType t4[] = {GOTO, SCAN, HOVER, RTH};
    MissionGraph g;

    printf("--- validator error cases ---\n");

    MissionGraph empty = {0};
    check("empty graph", s10_graph_validate(&empty), S10_GRAPH_EMPTY, -1);

    g = make_chain(t3, 3);
    check("valid chain", s10_graph_validate(&g), S10_GRAPH_OK, -1);
    s10_graph_free(&g);

    { PrimitiveType t[] = {TAKEOFF_CMD, RTH};
      g = make_chain(t, 2);
      check("takeoff + rth", s10_graph_validate(&g), S10_GRAPH_OK, -1);
      s10_graph_free(&g); }

    g = make_chain(t3, 3);
    g.nodes[1].id = 0;
    check("duplicate id", s10_graph_validate(&g), S10_GRAPH_DUPLICATE_ID, 0);
    s10_graph_free(&g);

    g = make_chain(t3, 3);
    g.nodes[2].id = 7;
    check("id != index", s10_graph_validate(&g), S10_GRAPH_BAD_ID, 7);
    s10_graph_free(&g);

    g = make_chain(t3, 3);
    g.edges[1].to_id = 9;
    check("edge to nonexistent node", s10_graph_validate(&g), S10_GRAPH_BAD_EDGE, 9);
    s10_graph_free(&g);

    g = make_chain(t4, 4);
    g.edges[1].from_id = 0; g.edges[1].to_id = 2;      // edges: 0->1, 0->2, 2->3
    check("branch (two out-edges)", s10_graph_validate(&g), S10_GRAPH_NOT_A_CHAIN, 0);
    s10_graph_free(&g);

    g = make_chain(t3, 3);
    g.edge_count = 1;
    check("wrong edge count", s10_graph_validate(&g), S10_GRAPH_NOT_A_CHAIN, -1);
    s10_graph_free(&g);

    g = make_chain(t3, 3);
    g.edges[0].from_id = 0; g.edges[0].to_id = 1;
    g.edges[1].from_id = 1; g.edges[1].to_id = 0;      // loop 0 <-> 1, node 2 detached
    check("cycle", s10_graph_validate(&g), S10_GRAPH_UNREACHABLE, 0);
    s10_graph_free(&g);

    g = make_chain(t4, 4);
    g.nodes[3].cmd.type = HOVER;
    check("no terminal node", s10_graph_validate(&g), S10_GRAPH_NO_TERMINAL, 3);
    s10_graph_free(&g);

    g = make_chain(t3, 3);
    g.nodes[0].cmd.position[0] = '\0';
    check("GOTO without position", s10_graph_validate(&g), S10_GRAPH_MISSING_PARAM, 0);
    s10_graph_free(&g);

    { PrimitiveType t[] = {HOVER, RTH};
      g = make_chain(t, 2);
      g.nodes[0].cmd.duration_s = 0;
      check("HOVER without duration", s10_graph_validate(&g), S10_GRAPH_MISSING_PARAM, 0);
      s10_graph_free(&g); }

    { PrimitiveType t[] = {PATROL_CMD, RTH};
      g = make_chain(t, 2);
      g.nodes[0].cmd.repetitions = 2;
      check("PATROL not unrolled", s10_graph_validate(&g), S10_GRAPH_MISSING_PARAM, 0);
      s10_graph_free(&g); }

    g = make_chain(t2, 2);
    { MissionGraph big = g;
      big.node_count = S10_MAX_NODES + 1;               // validator returns before touching arrays
      check("too many nodes", s10_graph_validate(&big), S10_GRAPH_TOO_LARGE, -1); }
    s10_graph_free(&g);
    printf("\n");
}

static void test_oversize_graph(void) {
    int n = S10_MAX_NODES + 1;
    Command *cmds = calloc((size_t)n, sizeof(Command));
    for (int i = 0; i < n; i++) cmds[i].type = RTH;
    MissionGraph g = build_mission(cmds, n);
    int ok = (g.node_count == 0 && g.nodes == NULL && g.edges == NULL);
    printf("build_mission with %d commands -> empty graph\n  %s\n\n", n, ok ? "PASS" : "FAIL");
    if (!ok) failures++;
    free(cmds);
    s10_graph_free(&g);
}

int main(void) {
    // original intents (regression)
    run("Survey North Field", 3, GOTO, RTH, S10_GRAPH_OK);
    run("deliver from Toronto to Berlin carrying medical supplies", 7, GOTO, RTH, S10_GRAPH_OK);
    run("deliver from A to B", 7, GOTO, RTH, S10_GRAPH_OK);          // payload optional
    run("please patrol Zone Alpha", 2, PATROL_CMD, RTH, S10_GRAPH_OK);
    run("return to base", 1, RTH, RTH, S10_GRAPH_OK);
    run("land", 1, LAND_CMD, LAND_CMD, S10_GRAPH_OK);
    run("land Helipad 2", 2, GOTO, LAND_CMD, S10_GRAPH_OK);
    run("fly around randomly", 0, -1, -1, S10_GRAPH_EMPTY);
    run("survey", 3, GOTO, RTH, S10_GRAPH_MISSING_PARAM);            // no area given

    // new intents
    run("take off", 2, TAKEOFF_CMD, RTH, S10_GRAPH_OK);
    run("loiter at Alpha for 20 seconds", 3, GOTO, RTH, S10_GRAPH_OK);
    run("hover for 2 minutes", 2, HOVER, RTH, S10_GRAPH_OK);
    run("hold position", 2, HOVER, RTH, S10_GRAPH_OK);

    // patrol unrolling
    run("patrol perimeter 3 times", 4, PATROL_CMD, RTH, S10_GRAPH_OK);
    run("patrol north fence for 2 laps", 3, PATROL_CMD, RTH, S10_GRAPH_OK);
    run("patrol A twice", 3, PATROL_CMD, RTH, S10_GRAPH_OK);
    run("patrol A 500 times", 0, -1, -1, S10_GRAPH_EMPTY);

    test_oversize_graph();
    test_validator_errors();

    printf("%s (%d failure%s)\n", failures ? "SOME TESTS FAILED" : "ALL TESTS PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}