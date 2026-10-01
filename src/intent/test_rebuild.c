#include "intent_parser.h"
#include "command_interpreter.h"
#include "mission_builder.h"
#include <stdio.h>
#include <stdlib.h>

static int failures = 0;

// parse -> interpret -> build the starting graph from a text mission
static MissionGraph base(const char *text) {
    int n = 0;
    Command *c = interpret_intent(parse_intent(text), &n);
    MissionGraph g = build_mission(c, n);
    free(c);
    return g;
}

// Applies one event at node `cur` and checks status, node count and last node type
// (exp_last of -1 skips the type check).
static void t(const char *name, const char *mission, int cur, S10ChangeEvent ev,
              S10ApplyStatus exp_st, int exp_nodes, int exp_last) {
    MissionGraph g = base(mission), out;
    S10ApplyStatus st = s10_graph_apply_change(&g, cur, &ev, &out);
    int ok = (st == exp_st) && (out.node_count == exp_nodes);
    if (ok && exp_nodes > 0 && exp_last >= 0)
        ok = ((int)out.nodes[exp_nodes - 1].cmd.type == exp_last);
    printf("%-34s status=%d nodes=%d  %s\n", name, st, out.node_count, ok ? "PASS" : "FAIL");
    if (!ok) failures++;
    s10_graph_free(&g);
    s10_graph_free(&out);
}

int main(void) {
    const char *D = "deliver from A to B";   // GOTO A, HOVER, PICKUP, GOTO B, HOVER, DROPOFF, RTH
    S10ChangeEvent low   = { S10_CHANGE_WEATHER,      S10_SEV_LOW,      -1, "" };
    S10ChangeEvent med   = { S10_CHANGE_WEATHER,      S10_SEV_MEDIUM,   -1, "" };
    S10ChangeEvent blkB  = { S10_CHANGE_AREA_BLOCKED, S10_SEV_HIGH,     -1, "b" };
    S10ChangeEvent node5 = { S10_CHANGE_OTHER,        S10_SEV_HIGH,      5, "" };
    S10ChangeEvent bat   = { S10_CHANGE_BATTERY,      S10_SEV_CRITICAL, -1, "" };
    S10ChangeEvent comm  = { S10_CHANGE_COMMS,        S10_SEV_CRITICAL, -1, "" };
    S10ChangeEvent blkNF = { S10_CHANGE_AREA_BLOCKED, S10_SEV_HIGH,     -1, "North Field" };

    t("low: unchanged",               D, 2, low,   S10_APPLY_UNCHANGED, 7, RTH);
    t("medium: hold inserted",        D, 2, med,   S10_APPLY_CHANGED,   8, RTH);
    t("high: area B ahead, cut",      D, 0, blkB,  S10_APPLY_CHANGED,   4, RTH);
    t("high: area B already passed",  D, 4, blkB,  S10_APPLY_UNCHANGED, 7, RTH);
    t("high: node 5 affected",        D, 1, node5, S10_APPLY_CHANGED,   6, RTH);
    t("critical battery: land",       D, 3, bat,   S10_APPLY_CHANGED,   4, LAND_CMD);
    t("critical comms: rth",          D, 3, comm,  S10_APPLY_CHANGED,   4, RTH);
    t("survey, area blocked at start", "survey North Field", 0, blkNF,
      S10_APPLY_CHANGED, 1, RTH);
    t("bad current node",             D, 7, low,   S10_APPLY_BAD_ARGS,  0, -1);

    printf("%s (%d failure%s)\n", failures ? "SOME TESTS FAILED" : "ALL TESTS PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}