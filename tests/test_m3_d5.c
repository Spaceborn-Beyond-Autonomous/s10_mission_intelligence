/*
 * Day 5 tests, Member 3: scenario validation of optimization and replanning.
 *
 * Whole missions (delivery, survey, patrol) are planned and optimized from the
 * real scenarios.yaml / objectives.yaml, then changes are injected and the
 * remainder is replanned. Properties are checked by independent code (sampled
 * clearance, recomputed sums) rather than by the planner's own maths.
 *
 * Usage: test_m3_d5 [scenarios.yaml] [objectives.yaml]
 *        (defaults: config/scenarios.yaml config/objectives.yaml)
 * Link with the Member 2 sources (mission_builder, graph validator, ...).
 */
#include "replanner.h"
#include "route_optimizer.h"
#include "route_planner.h"
#include "graph_planner.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(desc, cond) do { \
    if (cond) { g_pass++; printf("  [PASS] %s\n", desc); } \
    else      { g_fail++; printf("  [FAIL] %s\n", desc); } \
} while (0)

#define NEAR(a, b) (fabs((a) - (b)) < 1e-6)

static const char *g_scn = "config/scenarios.yaml";
static const char *g_obj = "config/objectives.yaml";
static ScenarioTable   g_t;
static ObjectiveConfig g_cfg;

/* ---------------- helpers ---------------- */

static Command cmd(PrimitiveType type, const char *position, const char *area,
                   const char *route, int duration_s)
{
    Command c;
    memset(&c, 0, sizeof(c));
    c.type = type;
    if (position) strncpy(c.position, position, sizeof(c.position) - 1);
    if (area)     strncpy(c.area, area, sizeof(c.area) - 1);
    if (route)    strncpy(c.route_or_area, route, sizeof(c.route_or_area) - 1);
    c.duration_s = duration_s;
    return c;
}

static MissionGraph delivery(void)
{
    Command c[7];
    c[0] = cmd(TAKEOFF_CMD, NULL, NULL, NULL, 0);
    c[1] = cmd(GOTO, "warehouse", NULL, NULL, 0);
    c[2] = cmd(PICKUP, "warehouse", NULL, NULL, 0);
    c[3] = cmd(GOTO, "zone b", NULL, NULL, 0);
    c[4] = cmd(DROPOFF, "zone b", NULL, NULL, 0);
    c[5] = cmd(RTH, NULL, NULL, NULL, 0);
    c[6] = cmd(LAND_CMD, NULL, NULL, NULL, 0);
    return build_mission(c, 7);
}

static MissionGraph survey(void)
{
    Command c[5];
    c[0] = cmd(TAKEOFF_CMD, NULL, NULL, NULL, 0);
    c[1] = cmd(GOTO, "north field", NULL, NULL, 0);
    c[2] = cmd(SCAN, NULL, "north field", NULL, 0);
    c[3] = cmd(RTH, NULL, NULL, NULL, 0);
    c[4] = cmd(LAND_CMD, NULL, NULL, NULL, 0);
    return build_mission(c, 5);
}

static MissionGraph patrol(const char *route)
{
    Command c[5];
    c[0] = cmd(TAKEOFF_CMD, NULL, NULL, NULL, 0);
    c[1] = cmd(PATROL_CMD, NULL, NULL, route, 0);
    c[2] = cmd(PATROL_CMD, NULL, NULL, route, 0);
    c[3] = cmd(RTH, NULL, NULL, NULL, 0);
    c[4] = cmd(LAND_CMD, NULL, NULL, NULL, 0);
    return build_mission(c, 5);
}

/* Independent clearance check: sample every leg every 0.25 m. */
static int route_clear(const Route *r, const S10Vec3 *start, const ScenarioTable *t)
{
    const S10Vec3 *p = start;
    int i;
    for (i = 0; i < r->waypoint_count; i++) {
        const S10Vec3 *q = &r->waypoints[i].position;
        double len = hypot(q->x - p->x, q->y - p->y);
        int steps = (int)(len / 0.25) + 1, s;
        for (s = 0; s <= steps; s++) {
            double f = (double)s / (double)steps;
            if (s10_point_in_no_fly(t, p->x + f * (q->x - p->x), p->y + f * (q->y - p->y)))
                return 0;
        }
        p = q;
    }
    return 1;
}

static int ids_valid(const Route *r, int node_count)
{
    int i;
    for (i = 0; i < r->waypoint_count; i++) {
        int id = r->waypoints[i].source_node_id;
        if (id != S10_NO_SOURCE_NODE && (id < 0 || id >= node_count)) return 0;
    }
    return 1;
}

static int count_action(const Route *r, PrimitiveType a)
{
    int i, n = 0;
    for (i = 0; i < r->waypoint_count; i++) if (r->waypoints[i].action == a) n++;
    return n;
}

static int count_synth(const Route *r)
{
    int i, n = 0;
    for (i = 0; i < r->waypoint_count; i++) if (r->waypoints[i].source_node_id == S10_NO_SOURCE_NODE) n++;
    return n;
}

static int same_route(const Route *a, const Route *b)
{
    int i;
    if (a->waypoint_count != b->waypoint_count) return 0;
    for (i = 0; i < a->waypoint_count; i++) {
        if (a->waypoints[i].position.x != b->waypoints[i].position.x ||
            a->waypoints[i].position.y != b->waypoints[i].position.y ||
            a->waypoints[i].position.z != b->waypoints[i].position.z ||
            a->waypoints[i].speed_mps != b->waypoints[i].speed_mps ||
            a->waypoints[i].source_node_id != b->waypoints[i].source_node_id) return 0;
    }
    return 1;
}

/* Field-by-field comparison of two waypoint arrays (every field). */
static int same_waypoints(const Waypoint *a, int na, const Waypoint *b, int nb)
{
    int i;
    if (na != nb) return 0;
    for (i = 0; i < na; i++) {
        if (a[i].source_node_id != b[i].source_node_id ||
            a[i].action != b[i].action ||
            a[i].position.x != b[i].position.x ||
            a[i].position.y != b[i].position.y ||
            a[i].position.z != b[i].position.z ||
            a[i].speed_mps != b[i].speed_mps ||
            a[i].hold_s != b[i].hold_s ||
            strcmp(a[i].label, b[i].label) != 0) return 0;
    }
    return 1;
}

/* The remainder starts at node k: only detour points precede its first
 * waypoint, and no waypoint refers to an earlier node. */
static int starts_at_node(const Route *r, int k)
{
    int idx = s10_route_find_waypoint_for_node(r, k), i;
    if (idx < 0) return 0;
    for (i = 0; i < r->waypoint_count; i++) {
        int id = r->waypoints[i].source_node_id;
        if (i < idx && id != S10_NO_SOURCE_NODE) return 0;
        if (id != S10_NO_SOURCE_NODE && id < k) return 0;
    }
    return 1;
}

static int opt(const MissionGraph *g, const ScenarioTable *t, const ObjectiveConfig *cfg,
               Route *best, S10OptSummary *s)
{
    S10RiskStubCtx ctx;
    ctx.table = t; ctx.cfg = cfg;
    return s10_optimize_route(g, t, cfg, s10_leg_risk_stub, &ctx, &t->home, best, s);
}

static S10ChangeEvent event(S10Severity sev, S10ChangeType type, int affected)
{
    S10ChangeEvent ev;
    memset(&ev, 0, sizeof(ev));
    ev.severity = sev; ev.type = type; ev.affected_node_id = affected;
    return ev;
}

static S10ReplanStatus replan(const MissionGraph *g, int cur, S10ChangeEvent ev, const ScenarioTable *t,
                              const ObjectiveConfig *cfg, const S10Vec3 *override,
                              MissionGraph *ng, Route *r, S10ReplanInfo *info)
{
    S10RiskStubCtx ctx;
    ctx.table = t; ctx.cfg = cfg;
    return s10_replan(g, cur, &ev, t, cfg, s10_leg_risk_stub, &ctx, override, ng, r, info);
}

static S10ReplanStatus replace(MissionGraph *g, Route *r, int cur, S10ChangeEvent ev,
                               const ScenarioTable *t, const ObjectiveConfig *cfg,
                               const S10Vec3 *override, S10ReplanInfo *info)
{
    S10RiskStubCtx ctx;
    ctx.table = t; ctx.cfg = cfg;
    return s10_replan_replace(g, r, cur, &ev, t, cfg, s10_leg_risk_stub, &ctx, override, info);
}

/* ---------------- optimization scenarios ---------------- */

static void test_delivery(void)
{
    MissionGraph g = delivery();
    Route best, again;
    S10OptSummary s, s2;
    int n, prev = -1, order_ok = 1, all_found = 1;

    printf("\n-- scenario: delivery (optimize whole mission) --\n");

    CHECK("optimization succeeds", opt(&g, &g_t, &g_cfg, &best, &s));
    CHECK("all 6 candidates feasible", s.candidates_feasible == 6);
    CHECK("route is feasible and costed", best.feasible == 1 && best.cost.total > 0.0);
    CHECK("route never enters a no-fly zone", route_clear(&best, &g_t.home, &g_t));
    CHECK("every waypoint maps to a real node or is a detour point", ids_valid(&best, g.node_count));
    for (n = 0; n < g.node_count; n++) {
        int idx = s10_route_find_waypoint_for_node(&best, n);
        if (idx < 0) all_found = 0;
        if (idx <= prev) order_ok = 0;
        prev = idx;
    }
    CHECK("every mission node is present in the route", all_found);
    CHECK("nodes appear in mission order", order_ok);
    CHECK("detour points were needed (route is longer than 7 waypoints)", best.waypoint_count > 7 && count_synth(&best) == best.waypoint_count - 7);
    CHECK("reproducible: same inputs, same route", opt(&g, &g_t, &g_cfg, &again, &s2) && same_route(&best, &again) && s.best_index == s2.best_index);
    s10_route_free(&again);
    s10_route_free(&best);
    s10_graph_free(&g);
}

static void test_survey(void)
{
    MissionGraph g = survey();
    Route best;
    S10OptSummary s;

    printf("\n-- scenario: survey --\n");

    CHECK("optimization succeeds", opt(&g, &g_t, &g_cfg, &best, &s));
    CHECK("the whole 21-point lawnmower is kept", count_action(&best, SCAN) == 21);
    CHECK("no detours needed: takeoff + goto + 21 scan + rth + land = 25", best.waypoint_count == 25 && count_synth(&best) == 0);
    CHECK("route never enters a no-fly zone", route_clear(&best, &g_t.home, &g_t));
    CHECK("direct and safe variants tie, so the lowest index (direct) wins", s.totals[0] == s.totals[1] && (s.best_index % 2) == 0);
    s10_route_free(&best);
    s10_graph_free(&g);
}

static void test_patrol(void)
{
    ScenarioTable t = g_t;
    MissionGraph g = patrol("perimeter");
    MissionGraph g2;
    Route best;
    S10OptSummary s;

    printf("\n-- scenario: patrol --\n");

    CHECK("the shipped 'perimeter' route touches the airport zone corner (200,200): optimization fails", !opt(&g, &g_t, &g_cfg, &best, &s));
    CHECK("  reason is TARGET_IN_NO_FLY at the patrol node (1)", s.variant_plan[0].status == S10_PLAN_TARGET_IN_NO_FLY && s.variant_plan[0].node_id == 1);
    CHECK("  nothing leaked: best is empty, no candidate feasible", best.waypoints == NULL && s.candidates_feasible == 0 && s.best_index == -1);
    s10_graph_free(&g);

    /* a route that stays clear of the zone */
    strcpy(t.routes[t.route_count].name, "inner");
    t.routes[t.route_count].point_count = 4;
    t.routes[t.route_count].pts[0].x = 0;   t.routes[t.route_count].pts[0].y = 0;
    t.routes[t.route_count].pts[1].x = 150; t.routes[t.route_count].pts[1].y = 0;
    t.routes[t.route_count].pts[2].x = 150; t.routes[t.route_count].pts[2].y = 150;
    t.routes[t.route_count].pts[3].x = 0;   t.routes[t.route_count].pts[3].y = 150;
    t.route_count++;

    g2 = patrol("inner");
    CHECK("a route clear of the zone plans fine", opt(&g2, &t, &g_cfg, &best, &s));
    CHECK("2 laps x 4 points + takeoff + rth + land = 11 waypoints, no detours", best.waypoint_count == 11 && count_synth(&best) == 0);
    CHECK("patrol waypoints keep the patrol node ids", count_action(&best, PATROL_CMD) == 8);
    CHECK("route never enters a no-fly zone", route_clear(&best, &t.home, &t));
    s10_route_free(&best);
    s10_graph_free(&g2);
}

static void test_weights(void)
{
    ScenarioTable t = g_t;
    MissionGraph gs = survey();
    MissionGraph gd = delivery();
    MissionGraph gp;
    ObjectiveConfig w;
    Route best;
    S10OptSummary s;

    printf("\n-- scenario: weights decide the winner --\n");

    strcpy(t.routes[t.route_count].name, "inner");
    t.routes[t.route_count].point_count = 4;
    t.routes[t.route_count].pts[0].x = 0;   t.routes[t.route_count].pts[0].y = 0;
    t.routes[t.route_count].pts[1].x = 150; t.routes[t.route_count].pts[1].y = 0;
    t.routes[t.route_count].pts[2].x = 150; t.routes[t.route_count].pts[2].y = 150;
    t.routes[t.route_count].pts[3].x = 0;   t.routes[t.route_count].pts[3].y = 150;
    t.route_count++;
    gp = patrol("inner");

    {
        MissionGraph *gs_all[3] = { &gs, &gd, &gp };
        const char *names[3] = { "survey", "delivery", "patrol" };
        char d[96];
        int m;

        for (m = 0; m < 3; m++) {
            w = g_cfg; w.w_fuel = 1; w.w_time = 0; w.w_risk = 0;
            opt(gs_all[m], &t, &w, &best, &s);
            snprintf(d, sizeof(d), "%s: fuel-only picks the slowest profile (eco, index 0)", names[m]);
            CHECK(d, s.best_index == 0);
            s10_route_free(&best);

            w = g_cfg; w.w_fuel = 0; w.w_time = 1; w.w_risk = 0;
            opt(gs_all[m], &t, &w, &best, &s);
            snprintf(d, sizeof(d), "%s: time-only picks the fastest profile (fast, index 4)", names[m]);
            CHECK(d, s.best_index == 4);
            s10_route_free(&best);
        }
    }

    w = g_cfg; w.w_fuel = 0; w.w_time = 0; w.w_risk = 1;
    opt(&gd, &t, &w, &best, &s);
    CHECK("delivery: risk-only picks a route that avoids the GPS-denied zone (risk 0)", best.cost.risk == 0.0 && (s.best_index % 2) == 1);
    s10_route_free(&best);

    w = g_cfg; w.w_fuel = 1; w.w_time = 0; w.w_risk = 0;
    opt(&gd, &t, &w, &best, &s);
    {
        double fuel_only_fuel = best.cost.fuel;
        s10_route_free(&best);
        w = g_cfg; w.w_fuel = 0; w.w_time = 1; w.w_risk = 0;
        opt(&gd, &t, &w, &best, &s);
        CHECK("delivery: the time-only winner burns more fuel than the fuel-only winner", best.cost.fuel > fuel_only_fuel);
        s10_route_free(&best);
    }

    s10_graph_free(&gs);
    s10_graph_free(&gd);
    s10_graph_free(&gp);
}

/* ---------------- resume / remainder scenarios ---------------- */

static void test_resume(void)
{
    MissionGraph g = delivery();
    Route full, rem;
    S10Vec3 sv;
    S10PlanResult res;
    CostBreakdown cf, cp, cr;
    int k, i;
    int all_suffix = 1, all_start = 1, all_add = 1;

    printf("\n-- scenario: resuming at every node of the delivery mission --\n");

    res = s10_plan_route(&g, &g_t, &g_t.home, &full);
    CHECK("full route plans", res.status == S10_PLAN_OK);
    CHECK("full route costed", s10_cost_evaluate(&full, &g_t.home, &g_cfg, NULL, NULL, &cf));

    for (k = 0; k < g.node_count; k++) {
        int idx, ok;
        res = s10_plan_route_from(&g, &g_t, k, NULL, &rem, &sv);
        if (res.status != S10_PLAN_OK) { all_suffix = 0; continue; }

        /* the remainder must be the tail of the full route, including the
         * detour points that lead into the resume node */
        idx = s10_route_find_waypoint_for_node(&full, k);
        while (idx > 0 && full.waypoints[idx - 1].source_node_id == S10_NO_SOURCE_NODE) idx--;

        ok = (rem.waypoint_count == full.waypoint_count - idx);
        for (i = 0; ok && i < rem.waypoint_count; i++) {
            const Waypoint *a = &rem.waypoints[i], *b = &full.waypoints[idx + i];
            ok = a->position.x == b->position.x && a->position.y == b->position.y &&
                 a->position.z == b->position.z && a->source_node_id == b->source_node_id &&
                 a->action == b->action && a->hold_s == b->hold_s;
        }
        if (!ok) all_suffix = 0;

        /* start = the real (non-detour) waypoint before the resume point, home for the first */
        {
            const S10Vec3 *expect = (idx > 0) ? &full.waypoints[idx - 1].position : &g_t.home;
            if (!(NEAR(sv.x, expect->x) && NEAR(sv.y, expect->y) && NEAR(sv.z, expect->z))) all_start = 0;
        }

        /* cost of the part flown + cost of the remainder = cost of the whole */
        if (idx > 0) {
            Route prefix;
            prefix = full;                       /* shallow view, not freed */
            prefix.waypoint_count = idx;
            if (!s10_cost_evaluate(&prefix, &g_t.home, &g_cfg, NULL, NULL, &cp) ||
                !s10_cost_evaluate(&rem, &sv, &g_cfg, NULL, NULL, &cr) ||
                !NEAR(cp.time_s + cr.time_s, cf.time_s) || !NEAR(cp.fuel + cr.fuel, cf.fuel)) all_add = 0;
        }
        s10_route_free(&rem);
    }
    CHECK("for every node k: remainder equals the tail of the full route", all_suffix);
    CHECK("for every node k: start = previous real waypoint (home for node 0)", all_start);
    CHECK("for every node k: time and fuel of flown part + remainder = whole route", all_add);

    s10_route_free(&full);
    s10_graph_free(&g);
}

static void test_resume_limits(void)
{
    MissionGraph g = delivery();
    MissionGraph sv_g = survey();
    Route rem;
    S10Vec3 sv;
    S10PlanResult res;

    printf("\n-- known limits, pinned (resume repeats the node's work, history counts) --\n");

    res = s10_plan_route_from(&g, &g_t, 2, NULL, &rem, &sv);
    CHECK("resume at PICKUP: the pickup (with its 10 s hold) is flown again", res.status == S10_PLAN_OK && rem.waypoints[0].action == PICKUP && rem.waypoints[0].hold_s == 10);
    s10_route_free(&rem);

    res = s10_plan_route_from(&g, &g_t, 4, NULL, &rem, &sv);
    CHECK("resume at DROPOFF: the dropoff (with its 10 s hold) is flown again", res.status == S10_PLAN_OK && rem.waypoints[0].action == DROPOFF && rem.waypoints[0].hold_s == 10);
    s10_route_free(&rem);

    res = s10_plan_route_from(&sv_g, &g_t, 2, NULL, &rem, &sv);
    CHECK("resume at SCAN: the whole pattern is flown again (21 points)", res.status == S10_PLAN_OK && count_action(&rem, SCAN) == 21);
    CHECK("  the remainder is those 21 + rth + land", rem.waypoint_count == 23);
    s10_route_free(&rem);

    s10_graph_free(&g);
    s10_graph_free(&sv_g);

    {   /* history counts toward the 256-waypoint cap */
        ScenarioTable t = g_t;
        Command c[40];
        MissionGraph big, bigger;
        S10ChangeEvent ev;
        MissionGraph ng;
        Route r;
        S10ReplanInfo info;
        int i, n = 0;

        t.no_fly_count = 0;                       /* no detours: only the cap matters */
        t.areas[0].rect = (S10Rect){0, 0, 100, 240};   /* 13 lines = 26 points per scan */

        for (i = 0; i < 9; i++)  c[n++] = cmd(SCAN, NULL, "north field", NULL, 0);
        for (i = 0; i < 22; i++) c[n++] = cmd(HOVER, NULL, NULL, NULL, 1);
        c[n++] = cmd(RTH, NULL, NULL, NULL, 0);
        big = build_mission(c, n);                /* 25 + 8*26 + 22 + 1 = 256 waypoints */

        res = s10_plan_route(&big, &t, &t.home, &r);
        CHECK("a mission of exactly 256 waypoints still plans", res.status == S10_PLAN_OK && r.waypoint_count == 256);
        s10_route_free(&r);

        res = s10_plan_route_from(&big, &t, big.node_count - 1, NULL, &r, &sv);
        CHECK("resuming at its last node leaves a 1-waypoint remainder", res.status == S10_PLAN_OK && r.waypoint_count == 1);
        s10_route_free(&r);

        c[n - 1] = cmd(HOVER, NULL, NULL, NULL, 1);   /* the extra hover goes where the RTH was... */
        c[n++]   = cmd(RTH, NULL, NULL, NULL, 0);     /* ...and the RTH stays last */
        bigger = build_mission(c, n);                 /* 257 waypoints */

        res = s10_plan_route_from(&bigger, &t, bigger.node_count - 1, NULL, &r, &sv);
        CHECK("one waypoint more: even a 1-waypoint remainder fails (history counts)", res.status == S10_PLAN_TOO_MANY_WAYPOINTS && r.waypoints == NULL);

        ev = event(S10_SEV_MEDIUM, S10_CHANGE_WEATHER, -1);
        {
            S10ReplanStatus rs = replan(&big, big.node_count - 1, ev, &t, &g_cfg, NULL, &ng, &r, &info);
            int good = (rs == S10_REPLAN_PLAN_FAILED && info.plan_result.status == S10_PLAN_TOO_MANY_WAYPOINTS);
            CHECK("the replanner reports it: MEDIUM adds a HOVER node and pushes the mission over the cap", good);
            if (!good) printf("         got replan=%d apply=%d plan=%d (expected replan=%d plan=%d)\n", (int)rs,
                              (int)info.apply_status, (int)info.plan_result.status, (int)S10_REPLAN_PLAN_FAILED, (int)S10_PLAN_TOO_MANY_WAYPOINTS);
        }
        CHECK("  and leaves the outputs empty", ng.nodes == NULL && r.waypoints == NULL);
        CHECK("  and the old graph is intact", big.node_count == 32);

        s10_graph_free(&big);
        s10_graph_free(&bigger);
    }
}

/* ---------------- replanning scenarios ---------------- */

static void test_replan_scenarios(void)
{
    MissionGraph g = delivery();
    MissionGraph ng, ngl;
    Route r, rl;
    S10ReplanInfo info, infol;
    S10ChangeEvent ev;
    int i;

    printf("\n-- scenario: replanning the delivery mission after a change --\n");

    /* MEDIUM vs LOW at node 3 (next: GOTO zone b) */
    ev = event(S10_SEV_LOW, S10_CHANGE_WEATHER, -1);
    CHECK("LOW at node 3: replans, graph unchanged", replan(&g, 3, ev, &g_t, &g_cfg, NULL, &ngl, &rl, &infol) == S10_REPLAN_OK && infol.apply_status == S10_APPLY_UNCHANGED);
    ev = event(S10_SEV_MEDIUM, S10_CHANGE_WEATHER, -1);
    CHECK("MEDIUM at node 3: replans, graph changed", replan(&g, 3, ev, &g_t, &g_cfg, NULL, &ng, &r, &info) == S10_REPLAN_OK && info.apply_status == S10_APPLY_CHANGED);
    CHECK("MEDIUM remainder starts with the 30 s HOVER", r.waypoints[0].action == HOVER && r.waypoints[0].hold_s == 30);
    CHECK("MEDIUM costs exactly 30 s more than LOW", NEAR(r.cost.time_s - rl.cost.time_s, 30.0));
    CHECK("MEDIUM burns exactly 30 x 0.5 = 15 more fuel than LOW", NEAR(r.cost.fuel - rl.cost.fuel, 15.0));
    CHECK("the same candidate wins in both (a constant added to every candidate)", info.opt.best_index == infol.opt.best_index);
    CHECK("remainder avoids the no-fly zone", route_clear(&r, &info.start_used, &g_t));
    CHECK("remainder ids refer to the NEW graph", ids_valid(&r, ng.node_count) && ng.node_count == 8);
    s10_graph_free(&ng); s10_route_free(&r);
    s10_graph_free(&ngl); s10_route_free(&rl);

    /* weights still decide after a change */
    {
        ObjectiveConfig w;
        ev = event(S10_SEV_LOW, S10_CHANGE_OTHER, -1);
        w = g_cfg; w.w_fuel = 1; w.w_time = 0; w.w_risk = 0;
        replan(&g, 3, ev, &g_t, &w, NULL, &ng, &r, &info);
        CHECK("after a change, fuel-only still picks eco (index 0)", info.opt.best_index == 0);
        s10_graph_free(&ng); s10_route_free(&r);
        w = g_cfg; w.w_fuel = 0; w.w_time = 1; w.w_risk = 0;
        replan(&g, 3, ev, &g_t, &w, NULL, &ng, &r, &info);
        CHECK("after a change, time-only still picks fast (index 4)", info.opt.best_index == 4);
        s10_graph_free(&ng); s10_route_free(&r);
    }

    /* HIGH: area blocked while the drone is at zone b -> return home past the airport zone */
    ev = event(S10_SEV_HIGH, S10_CHANGE_AREA_BLOCKED, 4);
    CHECK("HIGH at node 4: replans", replan(&g, 4, ev, &g_t, &g_cfg, NULL, &ng, &r, &info) == S10_REPLAN_OK);
    CHECK("  ends with RTH at home", r.waypoints[r.waypoint_count - 1].action == RTH && NEAR(r.waypoints[r.waypoint_count - 1].position.x, 0) && NEAR(r.waypoints[r.waypoint_count - 1].position.y, 0));
    CHECK("  the straight way home from zone b crosses the airport zone, so detours are added", count_synth(&r) >= 1);
    CHECK("  and the whole way home stays clear", route_clear(&r, &info.start_used, &g_t));
    s10_graph_free(&ng); s10_route_free(&r);

    /* CRITICAL battery: land where you are; CRITICAL other: go home */
    ev = event(S10_SEV_CRITICAL, S10_CHANGE_BATTERY, -1);
    CHECK("CRITICAL battery at node 4: replans", replan(&g, 4, ev, &g_t, &g_cfg, NULL, &ng, &r, &info) == S10_REPLAN_OK);
    CHECK("  one waypoint: LAND at ground level", r.waypoint_count == 1 && r.waypoints[0].action == LAND_CMD && NEAR(r.waypoints[0].position.z, 0));
    s10_graph_free(&ng); s10_route_free(&r);

    ev = event(S10_SEV_CRITICAL, S10_CHANGE_COMMS, -1);
    CHECK("CRITICAL comms at node 4: replans, ends at home", replan(&g, 4, ev, &g_t, &g_cfg, NULL, &ng, &r, &info) == S10_REPLAN_OK && r.waypoints[r.waypoint_count - 1].action == RTH);
    CHECK("  route home is clear", route_clear(&r, &info.start_used, &g_t));
    s10_graph_free(&ng); s10_route_free(&r);

    /* a NEW no-fly zone appears across the route that was winning */
    {
        ScenarioTable t2 = g_t;
        ObjectiveConfig one = g_cfg;
        double fuel_before;
        Route base;
        S10ReplanInfo base_info;
        MissionGraph base_g;

        ev = event(S10_SEV_LOW, S10_CHANGE_OTHER, -1);
        replan(&g, 3, ev, &g_t, &g_cfg, NULL, &base_g, &base, &base_info);

        strcpy(t2.no_fly[t2.no_fly_count].name, "new zone");
        t2.no_fly[t2.no_fly_count].rect = (S10Rect){260, 250, 300, 290};
        t2.no_fly_count++;

        CHECK("the new zone really cuts the old winning route", !route_clear(&base, &base_info.start_used, &t2));
        CHECK("new zone: replan still succeeds", replan(&g, 3, ev, &t2, &g_cfg, NULL, &ng, &r, &info) == S10_REPLAN_OK);
        CHECK("  graph unchanged (the zone arrives through the table)", info.apply_status == S10_APPLY_UNCHANGED);
        CHECK("  the replanned remainder is a different route", !same_route(&base, &r));
        CHECK("  and it is clear of BOTH zones", route_clear(&r, &info.start_used, &t2));
        s10_graph_free(&ng); s10_route_free(&r);

        /* with one speed profile and fuel-only weights, extra obstacles can only cost more fuel */
        one.profile_count = 1; one.profiles[0] = g_cfg.profiles[1];
        one.w_fuel = 1; one.w_time = 0; one.w_risk = 0;
        replan(&g, 3, ev, &g_t, &one, NULL, &ng, &r, &info);
        fuel_before = r.cost.fuel;
        s10_graph_free(&ng); s10_route_free(&r);
        replan(&g, 3, ev, &t2, &one, NULL, &ng, &r, &info);
        CHECK("  fuel-only, one profile: the new zone never makes the best route cheaper", r.cost.fuel >= fuel_before - 1e-9);
        s10_graph_free(&ng); s10_route_free(&r);

        s10_graph_free(&base_g); s10_route_free(&base);
    }

    /* real drone position given */
    {
        S10Vec3 here = {200, 60, 30};
        ev = event(S10_SEV_LOW, S10_CHANGE_OTHER, -1);
        CHECK("start override: replans from the given position", replan(&g, 3, ev, &g_t, &g_cfg, &here, &ng, &r, &info) == S10_REPLAN_OK);
        CHECK("  start_used = the given position", NEAR(info.start_used.x, 200) && NEAR(info.start_used.y, 60) && NEAR(info.start_used.z, 30));
        CHECK("  the first leg from there needs a detour and stays clear", r.waypoints[0].source_node_id == S10_NO_SOURCE_NODE && route_clear(&r, &here, &g_t));
        s10_graph_free(&ng); s10_route_free(&r);
    }

    /* two changes in a row */
    {
        MissionGraph ng1, ng2;
        Route r1, r2;
        S10ReplanInfo i1, i2;

        ev = event(S10_SEV_MEDIUM, S10_CHANGE_WEATHER, -1);
        CHECK("chain 1: MEDIUM at node 2", replan(&g, 2, ev, &g_t, &g_cfg, NULL, &ng1, &r1, &i1) == S10_REPLAN_OK && ng1.node_count == 8);
        ev = event(S10_SEV_CRITICAL, S10_CHANGE_BATTERY, -1);
        CHECK("chain 2: CRITICAL battery on the new graph at node 5", replan(&ng1, 5, ev, &g_t, &g_cfg, NULL, &ng2, &r2, &i2) == S10_REPLAN_OK);
        CHECK("  second remainder ends with LAND and its ids fit the second graph", r2.waypoints[r2.waypoint_count - 1].action == LAND_CMD && ids_valid(&r2, ng2.node_count));
        CHECK("  neither earlier graph was modified", g.node_count == 7 && ng1.node_count == 8);
        s10_graph_free(&ng1); s10_route_free(&r1);
        s10_graph_free(&ng2); s10_route_free(&r2);
    }

    /* the old graph is never modified */
    {
        MissionGraph ref = delivery();
        int same = (g.node_count == ref.node_count);
        for (i = 0; same && i < g.node_count; i++) same = (g.nodes[i].cmd.type == ref.nodes[i].cmd.type);
        CHECK("old graph unchanged after all of the above", same);
        s10_graph_free(&ref);
    }

    s10_graph_free(&g);
}

/* ---------------- real drone position for emergency actions ---------------- */

static void test_real_position(void)
{
    MissionGraph g = delivery(), ng, ngl, g2;
    Route r, rl;
    S10ReplanInfo info, infol;
    S10ChangeEvent ev, evl;
    S10Vec3 real = {250, 320, 30}, sv;
    S10PlanResult pr;
    Command c[4];
    int i1, i2;

    printf("\n-- real position: positionless HOVER/LAND go where the drone is --\n");

    ev = event(S10_SEV_CRITICAL, S10_CHANGE_BATTERY, -1);
    CHECK("no override: LAND stays at the assumed point (150,40,0), as before",
          replan(&g, 3, ev, &g_t, &g_cfg, NULL, &ng, &r, &info) == S10_REPLAN_OK && r.waypoint_count == 1 &&
          NEAR(r.waypoints[0].position.x, 150) && NEAR(r.waypoints[0].position.y, 40) && NEAR(r.waypoints[0].position.z, 0));
    s10_graph_free(&ng); s10_route_free(&r);

    CHECK("CRITICAL battery + real position: replans", replan(&g, 3, ev, &g_t, &g_cfg, &real, &ng, &r, &info) == S10_REPLAN_OK);
    CHECK("  one waypoint: LAND on the ground right below the drone (250,320,0)", r.waypoint_count == 1 && r.waypoints[0].action == LAND_CMD &&
          NEAR(r.waypoints[0].position.x, 250) && NEAR(r.waypoints[0].position.y, 320) && NEAR(r.waypoints[0].position.z, 0));
    CHECK("  no detour and no flight back: only the 30 m descent is costed", count_synth(&r) == 0 && NEAR(r.cost.time_s, 30.0 / r.waypoints[0].speed_mps));
    CHECK("  start_used is the real position", NEAR(info.start_used.x, 250) && NEAR(info.start_used.y, 320) && NEAR(info.start_used.z, 30));
    s10_graph_free(&ng); s10_route_free(&r);

    ev = event(S10_SEV_MEDIUM, S10_CHANGE_WEATHER, -1);
    evl = event(S10_SEV_LOW, S10_CHANGE_WEATHER, -1);
    replan(&g, 3, ev, &g_t, &g_cfg, &real, &ng, &r, &info);
    replan(&g, 3, evl, &g_t, &g_cfg, &real, &ngl, &rl, &infol);
    CHECK("MEDIUM + real position: the 30 s HOVER is at the drone (250,320,30)", r.waypoints[0].action == HOVER && r.waypoints[0].hold_s == 30 &&
          NEAR(r.waypoints[0].position.x, 250) && NEAR(r.waypoints[0].position.y, 320) && NEAR(r.waypoints[0].position.z, 30));
    CHECK("  costs exactly 30 s and 15 fuel more than LOW from the same position", NEAR(r.cost.time_s - rl.cost.time_s, 30.0) && NEAR(r.cost.fuel - rl.cost.fuel, 15.0));
    CHECK("  and stays clear of the no-fly zone", route_clear(&r, &real, &g_t));
    s10_graph_free(&ng); s10_route_free(&r); s10_graph_free(&ngl); s10_route_free(&rl);

    /* explicit position is never moved */
    c[0] = cmd(TAKEOFF_CMD, NULL, NULL, NULL, 0); c[1] = cmd(HOVER, "zone b", NULL, NULL, 5); c[2] = cmd(RTH, NULL, NULL, NULL, 0);
    g2 = build_mission(c, 3);
    pr = s10_plan_route_from(&g2, &g_t, 1, &real, &r, &sv);
    i1 = s10_route_find_waypoint_for_node(&r, 1);
    CHECK("a HOVER with its own position (zone b) is never moved", pr.status == S10_PLAN_OK && i1 >= 0 && NEAR(r.waypoints[i1].position.x, 400) && NEAR(r.waypoints[i1].position.y, 300));
    s10_route_free(&r); s10_graph_free(&g2);

    /* only the first waypoint of the resume node is moved (documented limit) */
    c[0] = cmd(GOTO, "warehouse", NULL, NULL, 0); c[1] = cmd(HOVER, NULL, NULL, NULL, 5); c[2] = cmd(HOVER, NULL, NULL, NULL, 5); c[3] = cmd(RTH, NULL, NULL, NULL, 0);
    g2 = build_mission(c, 4);
    pr = s10_plan_route_from(&g2, &g_t, 1, &real, &r, &sv);
    i1 = s10_route_find_waypoint_for_node(&r, 1); i2 = s10_route_find_waypoint_for_node(&r, 2);
    CHECK("two positionless HOVERs: the first (resume node) moves to the drone", pr.status == S10_PLAN_OK && i1 >= 0 && NEAR(r.waypoints[i1].position.x, 250) && NEAR(r.waypoints[i1].position.y, 320));
    CHECK("  the second one stays at the assumed point (150,40,30): known limit", i2 >= 0 && NEAR(r.waypoints[i2].position.x, 150) && NEAR(r.waypoints[i2].position.y, 40));
    s10_route_free(&r); s10_graph_free(&g2);

    /* real position inside a no-fly zone */
    {
        S10Vec3 in_nf = {250, 150, 30};
        CHECK("CRITICAL battery with the drone inside a no-fly zone: refused, reason TARGET_IN_NO_FLY",
              replan(&g, 3, ev = event(S10_SEV_CRITICAL, S10_CHANGE_BATTERY, -1), &g_t, &g_cfg, &in_nf, &ng, &r, &info) == S10_REPLAN_PLAN_FAILED &&
              info.plan_result.status == S10_PLAN_TARGET_IN_NO_FLY && ng.nodes == NULL && r.waypoints == NULL);
        CHECK("LOW (next node is a GOTO) from inside the zone: unchanged, NO_PATH",
              replan(&g, 1, event(S10_SEV_LOW, S10_CHANGE_OTHER, -1), &g_t, &g_cfg, &in_nf, &ng, &r, &info) == S10_REPLAN_PLAN_FAILED &&
              info.plan_result.status == S10_PLAN_NO_PATH);
    }

    /* real position inside a GPS-denied zone: landing in place is still fine */
    {
        S10Vec3 in_gd = {350, 200, 30};
        CHECK("CRITICAL battery inside the GPS-denied zone: replans",
              replan(&g, 3, event(S10_SEV_CRITICAL, S10_CHANGE_BATTERY, -1), &g_t, &g_cfg, &in_gd, &ng, &r, &info) == S10_REPLAN_OK);
        CHECK("  lands in place (350,200,0)", r.waypoint_count == 1 && NEAR(r.waypoints[0].position.x, 350) && NEAR(r.waypoints[0].position.y, 200) && NEAR(r.waypoints[0].position.z, 0));
        CHECK("  the 'safe' candidates are infeasible (the target is inside their no-fly copy), a direct one wins",
              info.opt.totals[1] < 0 && info.opt.totals[3] < 0 && info.opt.totals[5] < 0 && (info.opt.best_index % 2) == 0);
        s10_graph_free(&ng); s10_route_free(&r);
    }

    s10_graph_free(&g);
}

static void test_replan_sweep(void)
{
    MissionGraph g = delivery();
    static const S10ChangeType types[5] = { S10_CHANGE_WEATHER, S10_CHANGE_AREA_BLOCKED, S10_CHANGE_BATTERY, S10_CHANGE_COMMS, S10_CHANGE_OTHER };
    static const S10Severity sevs[4] = { S10_SEV_LOW, S10_SEV_MEDIUM, S10_SEV_HIGH, S10_SEV_CRITICAL };
    int ty, sv, cur;
    int total = 0, ok = 0, apply_failed = 0, other_fail = 0;
    int bad_clear = 0, bad_ids = 0, bad_feasible = 0, bad_cost = 0;

    printf("\n-- sweep: every change type x severity x resume node (140 replans) --\n");

    for (ty = 0; ty < 5; ty++) {
        for (sv = 0; sv < 4; sv++) {
            for (cur = 0; cur < g.node_count; cur++) {
                MissionGraph ng;
                Route r;
                S10ReplanInfo info;
                S10ChangeEvent ev = event(sevs[sv], types[ty], (types[ty] == S10_CHANGE_AREA_BLOCKED) ? cur : -1);
                S10ReplanStatus st = replan(&g, cur, ev, &g_t, &g_cfg, NULL, &ng, &r, &info);
                total++;
                if (st == S10_REPLAN_OK) {
                    ok++;
                    if (!route_clear(&r, &info.start_used, &g_t)) bad_clear++;
                    if (!ids_valid(&r, ng.node_count)) bad_ids++;
                    if (!r.feasible || r.waypoint_count < 1) bad_feasible++;
                    if (!(r.cost.total >= 0.0) || !(r.cost.time_s >= 0.0)) bad_cost++;
                    s10_graph_free(&ng);
                    s10_route_free(&r);
                } else if (st == S10_REPLAN_APPLY_FAILED) {
                    apply_failed++;
                    if (ng.nodes != NULL || r.waypoints != NULL) other_fail++;
                } else {
                    other_fail++;
                }
            }
        }
    }

    printf("  (%d replans: %d ok, %d apply-failed, %d other)\n", total, ok, apply_failed, other_fail);
    CHECK("all 140 combinations were exercised", total == 140);
    CHECK("no replan fails in planning or scoring (only the graph step may refuse)", other_fail == 0);
    CHECK("most replans succeed", ok > total / 2);
    CHECK("every successful remainder is clear of the no-fly zone", bad_clear == 0);
    CHECK("every successful remainder only refers to nodes of the new graph", bad_ids == 0);
    CHECK("every successful remainder is feasible and non-empty", bad_feasible == 0);
    CHECK("every successful remainder has a finite, non-negative cost", bad_cost == 0);

    s10_graph_free(&g);
}

/* ---------------- atomic plan replacement ---------------- */

static void test_replace(void)
{
    MissionGraph g, g2;
    Route r, r2;
    S10ReplanInfo info;
    S10ChangeEvent ev;

    printf("\n-- replace: atomic replacement of the active graph + route --\n");

    /* 1. LOW at node 3: graph unchanged, route becomes the remainder */
    g = delivery();
    memset(&r, 0, sizeof(r));
    s10_plan_route(&g, &g_t, &g_t.home, &r);
    ev = event(S10_SEV_LOW, S10_CHANGE_WEATHER, -1);
    CHECK("LOW at node 3: replace is OK", replace(&g, &r, 3, ev, &g_t, &g_cfg, NULL, &info) == S10_REPLAN_OK);
    CHECK("  graph still has 7 nodes", g.node_count == 7);
    CHECK("  route is the remainder: it starts at node 3", starts_at_node(&r, 3) && r.waypoint_count < 12);
    CHECK("  route is costed, clear, and its ids fit the graph", r.cost.total > 0.0 && route_clear(&r, &info.start_used, &g_t) && ids_valid(&r, g.node_count));
    s10_graph_free(&g); s10_route_free(&r);

    /* 2. MEDIUM at node 3: HOVER node inserted */
    g = delivery();
    memset(&r, 0, sizeof(r));
    s10_plan_route(&g, &g_t, &g_t.home, &r);
    ev = event(S10_SEV_MEDIUM, S10_CHANGE_WEATHER, -1);
    CHECK("MEDIUM at node 3: replace is OK", replace(&g, &r, 3, ev, &g_t, &g_cfg, NULL, &info) == S10_REPLAN_OK);
    CHECK("  graph now has 8 nodes", g.node_count == 8);
    CHECK("  route starts with the 30 s HOVER", r.waypoint_count > 0 && r.waypoints[0].action == HOVER && r.waypoints[0].hold_s == 30);
    CHECK("  ids refer to the new graph", ids_valid(&r, g.node_count));
    s10_graph_free(&g); s10_route_free(&r);

    /* 3. failure keeps the plan, byte for byte */
    {
        S10Vec3 in_nf = {250, 150, 30};
        MissionNode *nodes_ptr;
        MissionEdge *edges_ptr;
        Waypoint *wp_ptr;
        MissionNode *nodes_copy;
        MissionEdge *edges_copy;
        Waypoint *wp_copy;
        int node_count, edge_count, wp_count, same;

        g = delivery();
        memset(&r, 0, sizeof(r));
        CHECK("failure test: whole mission planned into the active route", s10_plan_route(&g, &g_t, &g_t.home, &r).status == S10_PLAN_OK && r.waypoint_count > 0);

        nodes_ptr = g.nodes; edges_ptr = g.edges; wp_ptr = r.waypoints;
        node_count = g.node_count; edge_count = g.edge_count; wp_count = r.waypoint_count;
        nodes_copy = malloc((size_t)node_count * sizeof(MissionNode));
        edges_copy = malloc((size_t)(edge_count > 0 ? edge_count : 1) * sizeof(MissionEdge));
        wp_copy    = malloc((size_t)wp_count * sizeof(Waypoint));
        memcpy(nodes_copy, g.nodes, (size_t)node_count * sizeof(MissionNode));
        if (edge_count > 0) memcpy(edges_copy, g.edges, (size_t)edge_count * sizeof(MissionEdge));
        memcpy(wp_copy, r.waypoints, (size_t)wp_count * sizeof(Waypoint));

        ev = event(S10_SEV_CRITICAL, S10_CHANGE_BATTERY, -1);
        CHECK("CRITICAL battery from inside the no-fly zone: PLAN_FAILED",
              replace(&g, &r, 3, ev, &g_t, &g_cfg, &in_nf, &info) == S10_REPLAN_PLAN_FAILED);
        CHECK("  graph: same nodes/edges pointers and counts", g.nodes == nodes_ptr && g.edges == edges_ptr && g.node_count == node_count && g.edge_count == edge_count);
        CHECK("  graph: contents unchanged", memcmp(g.nodes, nodes_copy, (size_t)node_count * sizeof(MissionNode)) == 0 &&
              (edge_count == 0 || memcmp(g.edges, edges_copy, (size_t)edge_count * sizeof(MissionEdge)) == 0));
        CHECK("  route: same waypoints pointer and count", r.waypoints == wp_ptr && r.waypoint_count == wp_count);
        same = same_waypoints(r.waypoints, r.waypoint_count, wp_copy, wp_count) &&
               memcmp(r.waypoints, wp_copy, (size_t)wp_count * sizeof(Waypoint)) == 0;
        CHECK("  route: every waypoint field unchanged", same);
        CHECK("  route: feasible flag unchanged", r.feasible == 1);

        free(nodes_copy); free(edges_copy); free(wp_copy);
        s10_graph_free(&g); s10_route_free(&r);
    }

    /* 4. two replaces in a row on the same active pair */
    g = delivery();
    memset(&r, 0, sizeof(r));
    s10_plan_route(&g, &g_t, &g_t.home, &r);
    ev = event(S10_SEV_MEDIUM, S10_CHANGE_WEATHER, -1);
    CHECK("chain 1: MEDIUM at node 2 is OK, 8 nodes", replace(&g, &r, 2, ev, &g_t, &g_cfg, NULL, &info) == S10_REPLAN_OK && g.node_count == 8);
    ev = event(S10_SEV_CRITICAL, S10_CHANGE_BATTERY, -1);
    CHECK("chain 2: CRITICAL battery at node 5 on the result is OK", replace(&g, &r, 5, ev, &g_t, &g_cfg, NULL, &info) == S10_REPLAN_OK);
    CHECK("  ids valid for the final graph", ids_valid(&r, g.node_count));
    CHECK("  final route ends with LAND", r.waypoint_count > 0 && r.waypoints[r.waypoint_count - 1].action == LAND_CMD);
    s10_graph_free(&g); s10_route_free(&r);

    /* 5. NULL arguments and an empty active route */
    g = delivery();
    memset(&r, 0, sizeof(r));
    ev = event(S10_SEV_MEDIUM, S10_CHANGE_WEATHER, -1);
    {
        MissionNode *nodes_ptr = g.nodes;
        S10RiskStubCtx ctx;
        ctx.table = &g_t; ctx.cfg = &g_cfg;

        CHECK("NULL active graph -> BAD_ARGS", replace(NULL, &r, 3, ev, &g_t, &g_cfg, NULL, NULL) == S10_REPLAN_BAD_ARGS);
        CHECK("NULL active route -> BAD_ARGS", replace(&g, NULL, 3, ev, &g_t, &g_cfg, NULL, NULL) == S10_REPLAN_BAD_ARGS);
        CHECK("NULL event -> BAD_ARGS, active plan untouched",
              s10_replan_replace(&g, &r, 3, NULL, &g_t, &g_cfg, s10_leg_risk_stub, &ctx, NULL, NULL) == S10_REPLAN_BAD_ARGS &&
              g.nodes == nodes_ptr && g.node_count == 7 && r.waypoints == NULL && r.waypoint_count == 0);
    }
    CHECK("empty active route + valid graph: replace works", replace(&g, &r, 3, ev, &g_t, &g_cfg, NULL, NULL) == S10_REPLAN_OK);
    CHECK("  the empty route now holds the remainder", r.waypoints != NULL && r.waypoint_count > 0 && g.node_count == 8);
    s10_graph_free(&g); s10_route_free(&r);

    /* 6. determinism: same inputs twice, identical results */
    g = delivery(); g2 = delivery();
    memset(&r, 0, sizeof(r)); memset(&r2, 0, sizeof(r2));
    s10_plan_route(&g, &g_t, &g_t.home, &r);
    s10_plan_route(&g2, &g_t, &g_t.home, &r2);
    ev = event(S10_SEV_MEDIUM, S10_CHANGE_WEATHER, -1);
    CHECK("two identical replaces are both OK",
          replace(&g, &r, 3, ev, &g_t, &g_cfg, NULL, NULL) == S10_REPLAN_OK &&
          replace(&g2, &r2, 3, ev, &g_t, &g_cfg, NULL, NULL) == S10_REPLAN_OK);
    CHECK("  identical graphs", g.node_count == g2.node_count);
    CHECK("  identical routes, every field", same_waypoints(r.waypoints, r.waypoint_count, r2.waypoints, r2.waypoint_count));
    CHECK("  identical costs", r.cost.total == r2.cost.total && r.cost.time_s == r2.cost.time_s && r.cost.fuel == r2.cost.fuel);
    s10_graph_free(&g); s10_route_free(&r);
    s10_graph_free(&g2); s10_route_free(&r2);
}

int main(int argc, char **argv)
{
    if (argc > 1) g_scn = argv[1];
    if (argc > 2) g_obj = argv[2];

    if (!s10_scenario_load(g_scn, &g_t) || !s10_objectives_load(&g_cfg, g_obj)) {
        printf("cannot load '%s' / '%s' - run from the package root or pass both paths\n", g_scn, g_obj);
        return 2;
    }

    test_delivery();
    test_survey();
    test_patrol();
    test_weights();
    test_resume();
    test_resume_limits();
    test_replan_scenarios();
    test_real_position();
    test_replan_sweep();
    test_replace();

    printf("\n==================\n");
    printf("PASS: %d  FAIL: %d\n", g_pass, g_fail);
    printf("==================\n");

    return g_fail == 0 ? 0 : 1;
}