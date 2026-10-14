/*
 * Day 2 tests, Member 3: waypoint generator, graph planner, route planner.
 *
 * Deterministic: no randomness, exact expectations. The scenario file path
 * is argv[1] (default: config/scenarios.yaml). Expected numbers come from
 * the placeholder scenario shipped with it:
 *   warehouse (150,40), zone b (400,300), north field (300,500)-(500,700),
 *   no-fly "airport zone" (200,100)-(300,220), margin 10, alt 30, 8 m/s.
 */
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
#define CLOSE(a, b, tol) (fabs((a) - (b)) < (tol))

static const char *g_yaml = "config/scenarios.yaml";
static ScenarioTable g_t;

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

/* Builds a sequential chain graph; caller frees nodes/edges with free(). */
static MissionGraph make_graph(const Command *cmds, int n)
{
    MissionGraph g;
    int i;
    memset(&g, 0, sizeof(g));
    g.nodes = calloc((size_t)n, sizeof(MissionNode));
    g.edges = calloc((size_t)(n > 1 ? n - 1 : 1), sizeof(MissionEdge));
    g.node_count = n;
    g.edge_count = n - 1;
    for (i = 0; i < n; i++) { g.nodes[i].id = i; g.nodes[i].cmd = cmds[i]; }
    for (i = 0; i < n - 1; i++) { g.edges[i].from_id = i; g.edges[i].to_id = i + 1; }
    return g;
}

static void free_graph(MissionGraph *g) { free(g->nodes); free(g->edges); }

static double route_len(const Route *r, const S10Vec3 *start)
{
    double d = 0.0;
    const S10Vec3 *p = start;
    int i;
    for (i = 0; i < r->waypoint_count; i++) {
        d += hypot(r->waypoints[i].position.x - p->x, r->waypoints[i].position.y - p->y);
        p = &r->waypoints[i].position;
    }
    return d;
}

/* Independent check (sampling, not the planner's own maths): no leg of the
 * route passes through a no-fly rectangle. */
static int route_avoids_no_fly(const Route *r, const S10Vec3 *start, const ScenarioTable *t)
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

static void write_file(const char *path, const char *text)
{
    FILE *f = fopen(path, "w");
    if (f) { fputs(text, f); fclose(f); }
}

static const char *MIN_YAML =
    "frame: local_enu_m\nhome: {x: 0, y: 0, z: 0}\n"
    "defaults: {cruise_alt_m: 30, cruise_speed_mps: 8, pickup_hold_s: 10,\n"
    "  dropoff_hold_s: 10, scan_spacing_m: 20, detour_margin_m: 10}\n";

/* ---------------- Route helpers ---------------- */
static void test_route_helpers(void)
{
    Route r;
    printf("\n-- s10_route_free / find_waypoint_for_node --\n");

    s10_route_free(NULL);
    CHECK("free(NULL) is safe", 1);

    memset(&r, 0, sizeof(r));
    s10_route_free(&r);
    CHECK("free on empty route is safe", r.waypoints == NULL && r.waypoint_count == 0);

    r.waypoints = calloc(4, sizeof(Waypoint));
    r.waypoint_count = 4;
    r.waypoints[0].source_node_id = 5;
    r.waypoints[1].source_node_id = S10_NO_SOURCE_NODE;
    r.waypoints[2].source_node_id = 7;
    r.waypoints[3].source_node_id = 7;
    CHECK("first match is returned when nodes repeat", s10_route_find_waypoint_for_node(&r, 7) == 2);
    CHECK("no match -> -1", s10_route_find_waypoint_for_node(&r, 99) == -1);
    CHECK("sentinel never matches", s10_route_find_waypoint_for_node(&r, S10_NO_SOURCE_NODE) == -1);
    CHECK("NULL route -> -1", s10_route_find_waypoint_for_node(NULL, 5) == -1);

    r.feasible = 1;
    s10_route_free(&r);
    CHECK("free zeroes the struct", r.waypoints == NULL && r.waypoint_count == 0 && r.feasible == 0);
    s10_route_free(&r);
    CHECK("double free is safe", 1);
}

/* ---------------- Scenario loader ---------------- */
static void test_loader(void)
{
    ScenarioTable t;
    printf("\n-- s10_scenario_load --\n");

    CHECK("loads the real scenarios.yaml", s10_scenario_load(g_yaml, &t));
    CHECK("3 points", t.point_count == 3);
    CHECK("1 area, 1 route, 1 no-fly, 1 gps-denied",
          t.area_count == 1 && t.route_count == 1 && t.no_fly_count == 1 && t.gps_denied_count == 1);
    CHECK("point 'zone b' = (400,300)", strcmp(t.points[1].name, "zone b") == 0 && NEAR(t.points[1].x, 400) && NEAR(t.points[1].y, 300));
    CHECK("area rect = (300,500)-(500,700)", NEAR(t.areas[0].rect.x_min, 300) && NEAR(t.areas[0].rect.y_max, 700));
    CHECK("route 'perimeter' has 4 points, last = (0,200)", t.route_count == 1 && t.routes[0].point_count == 4 && NEAR(t.routes[0].pts[3].x, 0) && NEAR(t.routes[0].pts[3].y, 200));
    CHECK("no-fly rect = (200,100)-(300,220)", NEAR(t.no_fly[0].rect.x_min, 200) && NEAR(t.no_fly[0].rect.y_max, 220));
    CHECK("defaults loaded", NEAR(t.defaults.cruise_alt_m, 30) && NEAR(t.defaults.cruise_speed_mps, 8) && t.defaults.pickup_hold_s == 10 && NEAR(t.defaults.scan_spacing_m, 20) && NEAR(t.defaults.detour_margin_m, 10));

    CHECK("missing file -> false", !s10_scenario_load("no/such/file.yaml", &t));
    CHECK("missing file leaves table zeroed", t.point_count == 0 && t.defaults.cruise_speed_mps == 0.0);
    CHECK("NULL path -> false", !s10_scenario_load(NULL, &t));

    {
        char buf[1024];
        snprintf(buf, sizeof(buf), "%sareas:\n  bad: {x_min: 10, y_min: 0, x_max: 5, y_max: 9}\n", MIN_YAML);
        write_file("/tmp/s10_bad_rect.yaml", buf);
        CHECK("inverted rectangle -> false", !s10_scenario_load("/tmp/s10_bad_rect.yaml", &t));
        CHECK("failed load leaves table zeroed", t.area_count == 0 && t.defaults.cruise_speed_mps == 0.0);

        snprintf(buf, sizeof(buf), "%spoints:\n  a: {x: 1, y: 2}\n  a: {x: 3, y: 4}\n", MIN_YAML);
        write_file("/tmp/s10_dup.yaml", buf);
        CHECK("duplicate name -> false", !s10_scenario_load("/tmp/s10_dup.yaml", &t));

        snprintf(buf, sizeof(buf), "%spoints:\n  a: {x: 1, z: 2}\n", MIN_YAML);
        write_file("/tmp/s10_badkey.yaml", buf);
        CHECK("unknown field name -> false", !s10_scenario_load("/tmp/s10_badkey.yaml", &t));

        snprintf(buf, sizeof(buf), "%spoints:\n  a: {x: abc, y: 2}\n", MIN_YAML);
        write_file("/tmp/s10_badnum.yaml", buf);
        CHECK("non-numeric value -> false", !s10_scenario_load("/tmp/s10_badnum.yaml", &t));

        write_file("/tmp/s10_nodefaults.yaml", "frame: local_enu_m\npoints:\n  a: {x: 1, y: 2}\n");
        CHECK("missing defaults -> false", !s10_scenario_load("/tmp/s10_nodefaults.yaml", &t));

        snprintf(buf, sizeof(buf), "%spoints:\n  a: {x: 1, y: 2\n", MIN_YAML);
        write_file("/tmp/s10_broken.yaml", buf);
        CHECK("broken YAML syntax -> false", !s10_scenario_load("/tmp/s10_broken.yaml", &t));
    }
}

/* ---------------- Resolver ---------------- */
static void test_resolver(void)
{
    S10Resolved r;
    printf("\n-- s10_resolve_location --\n");

    CHECK("'Zone B' -> point", s10_resolve_location(&g_t, "Zone B", &r) && r.kind == S10_LOC_POINT && strcmp(g_t.points[r.index].name, "zone b") == 0);
    CHECK("'at Zone B' strips the filler word", s10_resolve_location(&g_t, "at Zone B", &r) && r.kind == S10_LOC_POINT);
    CHECK("'  to the   Zone B ' trims and collapses", s10_resolve_location(&g_t, "  to the   Zone B ", &r) && r.kind == S10_LOC_POINT);
    CHECK("'North Field' -> area", s10_resolve_location(&g_t, "North Field", &r) && r.kind == S10_LOC_AREA);
    CHECK("'perimeter' -> route", s10_resolve_location(&g_t, "perimeter", &r) && r.kind == S10_LOC_ROUTE);
    CHECK("unknown name -> false, kind NONE", !s10_resolve_location(&g_t, "zone c", &r) && r.kind == S10_LOC_NONE);
    CHECK("empty string -> false", !s10_resolve_location(&g_t, "", &r));
    CHECK("only fillers -> false", !s10_resolve_location(&g_t, "at the", &r));
    CHECK("NULL text -> false", !s10_resolve_location(&g_t, NULL, &r));
}

/* ---------------- Waypoint generator ---------------- */
static void test_generator(void)
{
    Route r;
    S10GenResult res;
    MissionGraph g;

    printf("\n-- s10_generate_waypoints --\n");

    {   /* SURVEY: GOTO area, SCAN area, RTH */
        Command c[3];
        c[0] = cmd(GOTO, "north field", NULL, NULL, 0);
        c[1] = cmd(SCAN, NULL, "north field", NULL, 0);
        c[2] = cmd(RTH, NULL, NULL, NULL, 0);
        g = make_graph(c, 3);
        res = s10_generate_waypoints(&g, &g_t, &r);
        CHECK("survey: status OK", res.status == S10_GEN_OK && r.feasible == 1);
        CHECK("survey: 1 goto + 21 scan + 1 rth = 23 waypoints", r.waypoint_count == 23);
        CHECK("survey: GOTO area lands on entry corner (300,500,30)", NEAR(r.waypoints[0].position.x, 300) && NEAR(r.waypoints[0].position.y, 500) && NEAR(r.waypoints[0].position.z, 30));
        CHECK("survey: first scan point skipped (already at entry corner)", r.waypoints[1].action == SCAN && NEAR(r.waypoints[1].position.x, 500) && NEAR(r.waypoints[1].position.y, 500));
        CHECK("survey: second line runs back west at y=520", NEAR(r.waypoints[2].position.x, 500) && NEAR(r.waypoints[2].position.y, 520) && NEAR(r.waypoints[3].position.x, 300) && NEAR(r.waypoints[3].position.y, 520));
        CHECK("survey: last scan point is the far corner (500,700)", NEAR(r.waypoints[21].position.x, 500) && NEAR(r.waypoints[21].position.y, 700));
        CHECK("survey: RTH at home, cruise altitude", r.waypoints[22].action == RTH && NEAR(r.waypoints[22].position.x, 0) && NEAR(r.waypoints[22].position.z, 30));
        CHECK("survey: scan waypoints keep their node id and no hold", r.waypoints[5].source_node_id == 1 && r.waypoints[5].hold_s == 0);
        CHECK("survey: speed = cruise speed", NEAR(r.waypoints[5].speed_mps, 8));
        CHECK("survey: cost left zeroed", NEAR(r.cost.total, 0) && NEAR(r.cost.fuel, 0));
        s10_route_free(&r);
        free_graph(&g);
    }

    {   /* DELIVER-like: takeoff, goto, pickup, goto, dropoff, rth, land */
        Command c[7];
        c[0] = cmd(TAKEOFF_CMD, NULL, NULL, NULL, 0);
        c[1] = cmd(GOTO, "at the warehouse", NULL, NULL, 0);
        c[2] = cmd(PICKUP, "warehouse", NULL, NULL, 0);
        c[3] = cmd(GOTO, "Zone B", NULL, NULL, 0);
        c[4] = cmd(DROPOFF, "zone b", NULL, NULL, 0);
        c[5] = cmd(RTH, NULL, NULL, NULL, 0);
        c[6] = cmd(LAND_CMD, NULL, NULL, NULL, 0);
        g = make_graph(c, 7);
        res = s10_generate_waypoints(&g, &g_t, &r);
        CHECK("deliver: status OK, 7 waypoints", res.status == S10_GEN_OK && r.waypoint_count == 7);
        CHECK("deliver: takeoff climbs in place to (0,0,30)", r.waypoints[0].action == TAKEOFF_CMD && NEAR(r.waypoints[0].position.x, 0) && NEAR(r.waypoints[0].position.z, 30));
        CHECK("deliver: warehouse resolved through filler words", NEAR(r.waypoints[1].position.x, 150) && NEAR(r.waypoints[1].position.y, 40));
        CHECK("deliver: pickup holds 10 s", r.waypoints[2].action == PICKUP && r.waypoints[2].hold_s == 10);
        CHECK("deliver: dropoff at zone b holds 10 s", NEAR(r.waypoints[4].position.x, 400) && r.waypoints[4].hold_s == 10);
        CHECK("deliver: land goes to z=0 at home", r.waypoints[6].action == LAND_CMD && NEAR(r.waypoints[6].position.z, 0) && NEAR(r.waypoints[6].position.x, 0));
        CHECK("deliver: label keeps the normalized name", strcmp(r.waypoints[3].label, "zone b") == 0);
        s10_route_free(&r);
        free_graph(&g);
    }

    {   /* PATROL route and PATROL area */
        Command c[3];
        c[0] = cmd(PATROL_CMD, NULL, NULL, "perimeter", 0);
        c[1] = cmd(PATROL_CMD, NULL, NULL, "north field", 0);
        c[2] = cmd(RTH, NULL, NULL, NULL, 0);
        g = make_graph(c, 3);
        res = s10_generate_waypoints(&g, &g_t, &r);
        CHECK("patrol: 4 route points + 4 area corners + rth = 9", res.status == S10_GEN_OK && r.waypoint_count == 9);
        CHECK("patrol: route points in file order", NEAR(r.waypoints[1].position.x, 200) && NEAR(r.waypoints[1].position.y, 0));
        CHECK("patrol: area patrol starts at (300,500)", NEAR(r.waypoints[4].position.x, 300) && NEAR(r.waypoints[4].position.y, 500));
        s10_route_free(&r);
        free_graph(&g);
    }

    {   /* HOVER with and without a position */
        Command c[3];
        c[0] = cmd(GOTO, "warehouse", NULL, NULL, 0);
        c[1] = cmd(HOVER, NULL, NULL, NULL, 45);
        c[2] = cmd(RTH, NULL, NULL, NULL, 0);
        g = make_graph(c, 3);
        res = s10_generate_waypoints(&g, &g_t, &r);
        CHECK("hover: stays where it was, holds 45 s", res.status == S10_GEN_OK && NEAR(r.waypoints[1].position.x, 150) && r.waypoints[1].hold_s == 45);
        s10_route_free(&r);
        free_graph(&g);
    }

    {   /* scan spacing that does not divide the area: last line clamps to the edge */
        ScenarioTable t = g_t;
        Command c[2];
        t.areas[0].rect.x_min = 0;  t.areas[0].rect.x_max = 100;
        t.areas[0].rect.y_min = 0;  t.areas[0].rect.y_max = 50;
        c[0] = cmd(SCAN, NULL, "north field", NULL, 0);
        c[1] = cmd(RTH, NULL, NULL, NULL, 0);
        g = make_graph(c, 2);
        res = s10_generate_waypoints(&g, &t, &r);
        CHECK("uneven scan: 4 lines (0,20,40,50), first point skipped = 7 + rth", res.status == S10_GEN_OK && r.waypoint_count == 8);
        CHECK("uneven scan: last line is clamped to y=50 and none exceed it", NEAR(r.waypoints[6].position.y, 50) && NEAR(r.waypoints[5].position.y, 50));
        s10_route_free(&r);
        free_graph(&g);
    }

    {   /* error cases */
        Command c[2];

        c[0] = cmd(GOTO, "zone c", NULL, NULL, 0);
        c[1] = cmd(RTH, NULL, NULL, NULL, 0);
        g = make_graph(c, 2);
        res = s10_generate_waypoints(&g, &g_t, &r);
        CHECK("unknown location: status + node id + name in detail", res.status == S10_GEN_UNKNOWN_LOCATION && res.node_id == 0 && strstr(res.detail, "zone c") != NULL);
        CHECK("unknown location: route left empty", r.waypoints == NULL && r.waypoint_count == 0 && r.feasible == 0);
        free_graph(&g);

        c[0] = cmd(GOTO, "", NULL, NULL, 0);
        g = make_graph(c, 2);
        res = s10_generate_waypoints(&g, &g_t, &r);
        CHECK("GOTO without position -> MISSING_PARAM", res.status == S10_GEN_MISSING_PARAM && res.node_id == 0);
        free_graph(&g);

        c[0] = cmd(SCAN, NULL, "warehouse", NULL, 0);
        g = make_graph(c, 2);
        res = s10_generate_waypoints(&g, &g_t, &r);
        CHECK("SCAN of a point (not an area) -> UNKNOWN_LOCATION", res.status == S10_GEN_UNKNOWN_LOCATION);
        free_graph(&g);

        c[0] = cmd(PATROL_CMD, NULL, NULL, "warehouse", 0);
        g = make_graph(c, 2);
        res = s10_generate_waypoints(&g, &g_t, &r);
        CHECK("PATROL of a single point -> UNKNOWN_LOCATION", res.status == S10_GEN_UNKNOWN_LOCATION);
        free_graph(&g);

        memset(&g, 0, sizeof(g));
        res = s10_generate_waypoints(&g, &g_t, &r);
        CHECK("empty graph -> BAD_GRAPH", res.status == S10_GEN_BAD_GRAPH);
        CHECK("NULL graph -> BAD_GRAPH", s10_generate_waypoints(NULL, &g_t, &r).status == S10_GEN_BAD_GRAPH);

        c[0] = cmd(GOTO, "warehouse", NULL, NULL, 0);
        g = make_graph(c, 2);
        g.edge_count = 0;
        res = s10_generate_waypoints(&g, &g_t, &r);
        CHECK("two nodes with no edge -> BAD_GRAPH", res.status == S10_GEN_BAD_GRAPH);
        free_graph(&g);
    }

    {   /* too many waypoints: many survey lines */
        ScenarioTable t = g_t;
        Command c[2];
        t.defaults.scan_spacing_m = 1.0;
        c[0] = cmd(SCAN, NULL, "north field", NULL, 0);
        c[1] = cmd(RTH, NULL, NULL, NULL, 0);
        g = make_graph(c, 2);
        res = s10_generate_waypoints(&g, &t, &r);
        CHECK("201 lines -> TOO_MANY_WAYPOINTS, route left empty", res.status == S10_GEN_TOO_MANY_WAYPOINTS && r.waypoints == NULL);
        free_graph(&g);
    }
}

/* ---------------- Graph planner ---------------- */
static void test_graph_planner(void)
{
    S10Detour d;
    S10Vec3 a, b;
    S10LegStatus st;

    printf("\n-- graph_planner --\n");

    CHECK("point inside no-fly", s10_point_in_no_fly(&g_t, 250, 150));
    CHECK("border counts as inside", s10_point_in_no_fly(&g_t, 200, 100));
    CHECK("point outside no-fly", !s10_point_in_no_fly(&g_t, 199, 150));

    a = (S10Vec3){0, 0, 30}; b = (S10Vec3){150, 40, 30};
    st = s10_plan_leg(&a, &b, &g_t, &d);
    CHECK("clear leg -> DIRECT with 0 detour points", st == S10_LEG_DIRECT && d.count == 0);

    a = (S10Vec3){150, 40, 30}; b = (S10Vec3){400, 300, 30};
    st = s10_plan_leg(&a, &b, &g_t, &d);
    CHECK("warehouse -> zone b crosses the zone -> DETOURED, 1 point", st == S10_LEG_DETOURED && d.count == 1);
    CHECK("detour goes via the shorter (south-east) corner (310,90)", d.count == 1 && NEAR(d.pts[0].x, 310) && NEAR(d.pts[0].y, 90));
    CHECK("detour keeps altitude", d.count == 1 && NEAR(d.pts[0].z, 30));
    CHECK("detour length = 396.1 m (vs 360.6 straight)", d.count == 1 && CLOSE(hypot(160, 50) + hypot(90, 210), 396.1, 0.1));

    a = (S10Vec3){400, 300, 30}; b = (S10Vec3){150, 40, 30};
    st = s10_plan_leg(&a, &b, &g_t, &d);
    CHECK("reverse leg also detours", st == S10_LEG_DETOURED && d.count >= 1);

    a = (S10Vec3){0, 0, 0}; b = (S10Vec3){250, 150, 0};
    CHECK("goal inside no-fly -> BLOCKED", s10_plan_leg(&a, &b, &g_t, &d) == S10_LEG_BLOCKED && d.count == 0);
    CHECK("start inside no-fly -> BLOCKED", s10_plan_leg(&b, &a, &g_t, &d) == S10_LEG_BLOCKED);
    CHECK("NULL argument -> BLOCKED", s10_plan_leg(NULL, &a, &g_t, &d) == S10_LEG_BLOCKED);

    {   /* altitude is interpolated along the detour */
        a = (S10Vec3){150, 40, 0}; b = (S10Vec3){400, 300, 40};
        st = s10_plan_leg(&a, &b, &g_t, &d);
        CHECK("detour z lies between start z and goal z", st == S10_LEG_DETOURED && d.pts[0].z > 0 && d.pts[0].z < 40);
    }

    {   /* two zones side by side force a longer chain, still clear */
        ScenarioTable t = g_t;
        t.no_fly_count = 2;
        strcpy(t.no_fly[1].name, "second");
        t.no_fly[1].rect = (S10Rect){320, 100, 420, 220};
        a = (S10Vec3){150, 40, 30}; b = (S10Vec3){400, 300, 30};
        st = s10_plan_leg(&a, &b, &t, &d);
        CHECK("two zones: a clear path is still found", st == S10_LEG_DETOURED && d.count >= 1);
    }

    {   /* a very wide wall: the route goes around one end */
        ScenarioTable t = g_t;
        t.no_fly_count = 1;
        t.no_fly[0].rect = (S10Rect){-1000, 100, 1000, 120};   /* huge wall across the map */
        a = (S10Vec3){0, 0, 30}; b = (S10Vec3){0, 300, 30};
        st = s10_plan_leg(&a, &b, &t, &d);
        CHECK("wide wall: detour around its ends is found", st == S10_LEG_DETOURED);
    }
}

/* ---------------- Route planner ---------------- */
static void test_route_planner(void)
{
    Route r;
    S10PlanResult res;
    MissionGraph g;
    S10Vec3 home = g_t.home;
    int i, detour_count = 0;

    printf("\n-- route_planner --\n");

    {
        Command c[7];
        c[0] = cmd(TAKEOFF_CMD, NULL, NULL, NULL, 0);
        c[1] = cmd(GOTO, "warehouse", NULL, NULL, 0);
        c[2] = cmd(PICKUP, "warehouse", NULL, NULL, 0);
        c[3] = cmd(GOTO, "zone b", NULL, NULL, 0);
        c[4] = cmd(DROPOFF, "zone b", NULL, NULL, 0);
        c[5] = cmd(RTH, NULL, NULL, NULL, 0);
        c[6] = cmd(LAND_CMD, NULL, NULL, NULL, 0);
        g = make_graph(c, 7);
        res = s10_plan_route(&g, &g_t, &home, &r);

        CHECK("deliver: planning succeeds", res.status == S10_PLAN_OK && r.feasible == 1);
        CHECK("deliver: detour points were inserted (more than 7 waypoints)", r.waypoint_count > 7);
        for (i = 0; i < r.waypoint_count; i++)
            if (r.waypoints[i].source_node_id == S10_NO_SOURCE_NODE) detour_count++;
        CHECK("deliver: every extra waypoint is a synthesized detour point", detour_count == r.waypoint_count - 7);
        CHECK("deliver: first detour sits before 'goto zone b' at (310,90,30)", r.waypoints[3].source_node_id == S10_NO_SOURCE_NODE && NEAR(r.waypoints[3].position.x, 310) && NEAR(r.waypoints[3].position.y, 90) && NEAR(r.waypoints[3].position.z, 30));
        CHECK("deliver: detour point is a plain pass-through (GOTO, no hold, no label)", r.waypoints[3].action == GOTO && r.waypoints[3].hold_s == 0 && r.waypoints[3].label[0] == '\0');
        CHECK("deliver: detour inherits the next waypoint's speed", NEAR(r.waypoints[3].speed_mps, r.waypoints[4].speed_mps));
        CHECK("deliver: no leg of the final route crosses the no-fly zone (independent check)", route_avoids_no_fly(&r, &home, &g_t));
        CHECK("deliver: original waypoints kept in order", r.waypoints[2].action == PICKUP && r.waypoints[r.waypoint_count - 1].action == LAND_CMD);
        CHECK("deliver: find waypoint for node 3 skips the detour point", s10_route_find_waypoint_for_node(&r, 3) == 4);
        CHECK("deliver: route is longer than the straight-line sum", route_len(&r, &home) > 150 + 360.0);
        s10_route_free(&r);
        free_graph(&g);
    }

    {   /* a mission that never crosses the zone needs no detours */
        Command c[3];
        c[0] = cmd(GOTO, "landing pad", NULL, NULL, 0);
        c[1] = cmd(GOTO, "warehouse", NULL, NULL, 0);
        c[2] = cmd(LAND_CMD, NULL, NULL, NULL, 0);
        g = make_graph(c, 3);
        res = s10_plan_route(&g, &g_t, &home, &r);
        CHECK("clear mission: 3 waypoints, no detours", res.status == S10_PLAN_OK && r.waypoint_count == 3);
        s10_route_free(&r);
        free_graph(&g);
    }

    {   /* target inside the no-fly zone */
        ScenarioTable t = g_t;
        Command c[2];
        t.points[0].x = 250; t.points[0].y = 150;      /* move 'warehouse' into the zone */
        c[0] = cmd(GOTO, "warehouse", NULL, NULL, 0);
        c[1] = cmd(RTH, NULL, NULL, NULL, 0);
        g = make_graph(c, 2);
        res = s10_plan_route(&g, &t, &home, &r);
        CHECK("target in no-fly -> TARGET_IN_NO_FLY with node id", res.status == S10_PLAN_TARGET_IN_NO_FLY && res.node_id == 0);
        CHECK("failed plan leaves the route empty and infeasible", r.waypoints == NULL && r.waypoint_count == 0 && r.feasible == 0);
        free_graph(&g);
    }

    {   /* generation failure is passed through with its node and reason */
        Command c[2];
        c[0] = cmd(GOTO, "nowhere", NULL, NULL, 0);
        c[1] = cmd(RTH, NULL, NULL, NULL, 0);
        g = make_graph(c, 2);
        res = s10_plan_route(&g, &g_t, &home, &r);
        CHECK("unknown name -> GENERATION_FAILED with node and detail", res.status == S10_PLAN_GENERATION_FAILED && res.node_id == 0 && strstr(res.detail, "nowhere") != NULL);
        free_graph(&g);
    }

    CHECK("NULL argument -> BAD_ARGUMENT", s10_plan_route(NULL, &g_t, &home, &r).status == S10_PLAN_BAD_ARGUMENT);
    CHECK("insert_detours on an empty route -> BAD_ARGUMENT", s10_insert_detours(&r, &home, &g_t).status == S10_PLAN_BAD_ARGUMENT);

    {   /* start position matters: starting beyond the zone changes the first leg */
        Command c[2];
        S10Vec3 far_start = {400, 300, 30};
        c[0] = cmd(GOTO, "warehouse", NULL, NULL, 0);
        c[1] = cmd(RTH, NULL, NULL, NULL, 0);
        g = make_graph(c, 2);
        res = s10_plan_route(&g, &g_t, &far_start, &r);
        CHECK("start at zone b: first leg to warehouse is detoured", res.status == S10_PLAN_OK && r.waypoints[0].source_node_id == S10_NO_SOURCE_NODE);
        CHECK("start at zone b: whole route stays clear of the zone", route_avoids_no_fly(&r, &far_start, &g_t));
        s10_route_free(&r);
        free_graph(&g);
    }
}

int main(int argc, char **argv)
{
    if (argc > 1) g_yaml = argv[1];

    if (!s10_scenario_load(g_yaml, &g_t)) {
        printf("cannot load scenario file '%s' - run from the package root or pass the path\n", g_yaml);
        return 2;
    }

    test_route_helpers();
    test_loader();
    test_resolver();
    test_generator();
    test_graph_planner();
    test_route_planner();

    printf("\n==================\n");
    printf("PASS: %d  FAIL: %d\n", g_pass, g_fail);
    printf("==================\n");

    return g_fail == 0 ? 0 : 1;
}