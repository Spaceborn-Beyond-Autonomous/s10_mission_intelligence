/**
 * @file    test_m3_d4.c
 * @brief   Day 4 smoke tests: resume planning, optimizer wrapper, replanner.
 *
 * Build it like test_day3 (same sources plus replanner.c, route_planner.c,
 * route_optimizer.c, mission_builder.c, graph_validator.c, graph_planner.c).
 * Usage: ./test_day4 [path/to/objectives.yaml]
 *
 * The scenario is built in code so the tests do not depend on
 * scenarios.yaml. The loader is s10_objectives_load(&cfg, path).
 */

 #include <math.h>
 #include <stdio.h>
 #include <string.h>
 
 #include "replanner.h"
 #include "route_optimizer.h"
 #include "route_planner.h"
 
 static int g_pass = 0;
 static int g_fail = 0;
 
 #define CHECK(cond, name)                                  \
     do {                                                   \
         if(cond) { g_pass++; printf("  [PASS] %s\n", name); } \
         else     { g_fail++; printf("  [FAIL] %s\n", name); } \
     } while(0)
 
 #define NEAR(a, b)  (fabs((a) - (b)) < 1e-6)
 
 /************************************* Helpers ************************************* */
 
 static void make_table(ScenarioTable *t)
 {
     memset(t, 0, sizeof(*t));
 
     t->defaults.cruise_alt_m = 30.0;
     t->defaults.cruise_speed_mps = 10.0;
     t->defaults.pickup_hold_s = 5;
     t->defaults.dropoff_hold_s = 5;
     t->defaults.scan_spacing_m = 20.0;
     t->defaults.detour_margin_m = 5.0;
 
     snprintf(t->points[0].name, S10_NAME_LEN, "a");
     t->points[0].x = 100.0;  t->points[0].y = 0.0;
     snprintf(t->points[1].name, S10_NAME_LEN, "b");
     t->points[1].x = 100.0;  t->points[1].y = 100.0;
     t->point_count = 2;
 }
 
 /* Mission: 0 GOTO a, 1 GOTO b, 2 RTH. */
 static MissionGraph make_graph(void)
 {
     Command cmds[3];
 
     memset(cmds, 0, sizeof(cmds));
     cmds[0].type = GOTO;  snprintf(cmds[0].position, sizeof(cmds[0].position), "a");
     cmds[1].type = GOTO;  snprintf(cmds[1].position, sizeof(cmds[1].position), "b");
     cmds[2].type = RTH;
 
     return (build_mission(cmds, 3));
 }
 
 static S10ChangeEvent make_event(S10Severity sev, S10ChangeType type)
 {
     S10ChangeEvent ev;
 
     memset(&ev, 0, sizeof(ev));
     ev.type = type;
     ev.severity = sev;
     ev.affected_node_id = -1;
 
     return (ev);
 }
 
 static bool same_vec(const S10Vec3 *a, const S10Vec3 *b)
 {
     return (NEAR(a->x, b->x) && NEAR(a->y, b->y) && NEAR(a->z, b->z));
 }
 
 /************************************* Tests ************************************* */
 
 int main(int argc, char **argv)
 {
     const char    *obj_path = (argc > 1) ? argv[1] : "config/objectives.yaml";
     ScenarioTable  t;
     ObjectiveConfig cfg;
     MissionGraph   g;
     S10ChangeEvent ev;
     MissionGraph   ng;
     Route          r;
     S10ReplanInfo  info;
     S10ReplanStatus st;
     S10Vec3        home;
     S10Vec3        sv;
 
     make_table(&t);
     g = make_graph();
     home = t.home;
 
     memset(&cfg, 0, sizeof(cfg));
     if(!s10_objectives_load(&cfg, obj_path))
     {
         printf("cannot load objectives from %s\n", obj_path);
         return (1);
     }
 
     printf("\n-- s10_plan_route_from --\n");
     {
         Route pr;
         S10PlanResult res;
 
         res = s10_plan_route_from(&g, &t, 1, NULL, &pr, &sv);
         CHECK(res.status == S10_PLAN_OK, "resume at node 1 plans");
         CHECK(pr.waypoint_count == 2, "remainder = GOTO b + RTH (2 waypoints)");
         CHECK(pr.waypoints[0].source_node_id == 1, "first waypoint belongs to the resume node");
         CHECK(NEAR(sv.x, 100.0) && NEAR(sv.y, 0.0), "start = end of node 0 (point a)");
         s10_route_free(&pr);
 
         res = s10_plan_route_from(&g, &t, 0, NULL, &pr, &sv);
         CHECK((res.status == S10_PLAN_OK) && (pr.waypoint_count == 3), "resume at 0 keeps everything");
         CHECK(same_vec(&sv, &home), "resume at 0: start = home");
         s10_route_free(&pr);
 
         res = s10_plan_route_from(&g, &t, 99, NULL, &pr, &sv);
         CHECK(res.status == S10_PLAN_RESUME_NODE_NOT_FOUND, "unknown resume node -> RESUME_NODE_NOT_FOUND");
         CHECK((pr.waypoints == NULL) && (pr.waypoint_count == 0), "not found: route left empty");
 
         res = s10_plan_route_from(&g, &t, -1, NULL, &pr, NULL);
         CHECK(res.status == S10_PLAN_BAD_ARGUMENT, "no resume and no start -> BAD_ARGUMENT");
 
         res = s10_plan_route(&g, &t, &home, &pr);
         CHECK((res.status == S10_PLAN_OK) && (pr.waypoint_count == 3), "s10_plan_route wrapper still plans the whole mission");
         s10_route_free(&pr);
     }
 
     printf("\n-- optimizer wrapper --\n");
     {
         Route best;
         S10OptSummary sum;
 
         CHECK(s10_optimize_route(&g, &t, &cfg, NULL, NULL, &home, &best, &sum), "s10_optimize_route still works");
         CHECK(best.waypoint_count == 3, "whole mission has 3 waypoints");
         s10_route_free(&best);
 
         CHECK(!s10_optimize_route(&g, &t, &cfg, NULL, NULL, NULL, &best, &sum), "NULL start -> false");
 
         CHECK(s10_optimize_route_from(&g, &t, &cfg, NULL, NULL, 1, NULL, &best, &sum, &sv), "optimize from node 1");
         CHECK(best.waypoint_count == 2, "remainder has 2 waypoints");
         CHECK(NEAR(sv.x, 100.0) && NEAR(sv.y, 0.0), "start_used = point a");
         s10_route_free(&best);
     }
 
     printf("\n-- replanner: severities --\n");
 
     ev = make_event(S10_SEV_LOW, S10_CHANGE_WEATHER);
     st = s10_replan(&g, 1, &ev, &t, &cfg, NULL, NULL, NULL, &ng, &r, &info);
     CHECK(st == S10_REPLAN_OK, "LOW: replan ok");
     CHECK(info.apply_status == S10_APPLY_UNCHANGED, "LOW: graph unchanged");
     CHECK(ng.node_count == 3, "LOW: graph still 3 nodes");
     CHECK((r.waypoint_count == 2) && (r.waypoints[0].source_node_id == 1), "LOW: remainder starts at node 1");
     CHECK(r.feasible && (r.cost.total > 0.0), "LOW: route feasible with a cost");
     s10_graph_free(&ng);  s10_route_free(&r);
 
     ev = make_event(S10_SEV_LOW, S10_CHANGE_WEATHER);
     st = s10_replan(&g, 0, &ev, &t, &cfg, NULL, NULL, NULL, &ng, &r, &info);
     CHECK((st == S10_REPLAN_OK) && same_vec(&info.start_used, &home), "cur = 0: start_used = home");
     s10_graph_free(&ng);  s10_route_free(&r);
 
     ev = make_event(S10_SEV_MEDIUM, S10_CHANGE_WEATHER);
     st = s10_replan(&g, 1, &ev, &t, &cfg, NULL, NULL, NULL, &ng, &r, &info);
     CHECK((st == S10_REPLAN_OK) && (info.apply_status == S10_APPLY_CHANGED), "MEDIUM: replan ok, graph changed");
     CHECK(ng.node_count == 4, "MEDIUM: one HOVER inserted");
     CHECK((r.waypoints[0].action == HOVER) && (r.waypoints[0].hold_s == 30), "MEDIUM: first waypoint is the 30 s HOVER");
     CHECK(r.waypoints[0].source_node_id == 1, "MEDIUM: HOVER carries the resume node id");
     s10_graph_free(&ng);  s10_route_free(&r);
 
     ev = make_event(S10_SEV_HIGH, S10_CHANGE_AREA_BLOCKED);
     ev.affected_node_id = 1;
     st = s10_replan(&g, 1, &ev, &t, &cfg, NULL, NULL, NULL, &ng, &r, &info);
     CHECK(st == S10_REPLAN_OK, "HIGH: replan ok");
     CHECK((ng.node_count == 2) && (ng.nodes[1].cmd.type == RTH), "HIGH: affected node cut, RTH appended");
     CHECK((r.waypoint_count == 1) && (r.waypoints[0].action == RTH), "HIGH: remainder is a single RTH");
     s10_graph_free(&ng);  s10_route_free(&r);
 
     ev = make_event(S10_SEV_CRITICAL, S10_CHANGE_BATTERY);
     st = s10_replan(&g, 1, &ev, &t, &cfg, NULL, NULL, NULL, &ng, &r, &info);
     CHECK((st == S10_REPLAN_OK) && (r.waypoint_count == 1), "CRITICAL battery: one waypoint left");
     CHECK(r.waypoints[0].action == LAND_CMD, "CRITICAL battery: LAND");
     s10_graph_free(&ng);  s10_route_free(&r);
 
     ev = make_event(S10_SEV_CRITICAL, S10_CHANGE_COMMS);
     st = s10_replan(&g, 1, &ev, &t, &cfg, NULL, NULL, NULL, &ng, &r, &info);
     CHECK((st == S10_REPLAN_OK) && (r.waypoints[0].action == RTH), "CRITICAL non-battery: RTH");
     s10_graph_free(&ng);  s10_route_free(&r);
 
     printf("\n-- replanner: updated constraints --\n");
     {
         ScenarioTable t2 = t;
         Route         plain;
         int           i;
         bool          has_detour = false;
 
         ev = make_event(S10_SEV_LOW, S10_CHANGE_OTHER);
         st = s10_replan(&g, 1, &ev, &t, &cfg, NULL, NULL, NULL, &ng, &plain, &info);
         CHECK(st == S10_REPLAN_OK, "baseline remainder (no zone) ok");
         s10_graph_free(&ng);
 
         /* New no-fly zone straddling the leg a -> b. */
         snprintf(t2.no_fly[0].name, S10_NAME_LEN, "new_zone");
         t2.no_fly[0].rect.x_min = 80.0;  t2.no_fly[0].rect.x_max = 120.0;
         t2.no_fly[0].rect.y_min = 40.0;  t2.no_fly[0].rect.y_max = 60.0;
         t2.no_fly_count = 1;
 
         st = s10_replan(&g, 1, &ev, &t2, &cfg, NULL, NULL, NULL, &ng, &r, &info);
         CHECK(info.apply_status == S10_APPLY_UNCHANGED, "zone change: graph itself unchanged");
         CHECK(st == S10_REPLAN_OK, "zone change: still replans");
         CHECK(r.waypoint_count > plain.waypoint_count, "zone change: detour points added");
 
         for(i = 0; i < r.waypoint_count; i++)
         {
             if(r.waypoints[i].source_node_id == S10_NO_SOURCE_NODE) { has_detour = true; }
         }
         CHECK(has_detour, "zone change: detour waypoints are synthesized (-1)");
         s10_graph_free(&ng);  s10_route_free(&r);
 
         /* Drone position inside the new zone: first leg cannot be planned. */
         {
             S10Vec3 inside = { 100.0, 50.0, 30.0 };
 
             st = s10_replan(&g, 1, &ev, &t2, &cfg, NULL, NULL, &inside, &ng, &r, &info);
             CHECK(st == S10_REPLAN_PLAN_FAILED, "start inside a no-fly zone -> PLAN_FAILED");
             CHECK(info.plan_result.status == S10_PLAN_NO_PATH, "  reason is NO_PATH");
             CHECK((ng.nodes == NULL) && (r.waypoints == NULL), "  outputs left empty");
         }
 
         s10_route_free(&plain);
     }
 
     printf("\n-- replanner: real-position override --\n");
     {
         S10Vec3 real = { 150.0, 40.0, 25.0 };

         ev = make_event(S10_SEV_CRITICAL, S10_CHANGE_BATTERY);
         st = s10_replan(&g, 1, &ev, &t, &cfg, NULL, NULL, &real, &ng, &r, &info);
         CHECK((st == S10_REPLAN_OK) && (r.waypoint_count == 1), "CRITICAL battery + override: one waypoint");
         CHECK((r.waypoints[0].action == LAND_CMD) && NEAR(r.waypoints[0].position.x, 150.0) &&
               NEAR(r.waypoints[0].position.y, 40.0), "LAND is under the real position");
         CHECK(NEAR(r.waypoints[0].position.z, 0.0), "LAND stays at z = 0");
         CHECK(same_vec(&info.start_used, &real), "start_used = the override");
         s10_graph_free(&ng);  s10_route_free(&r);

         ev = make_event(S10_SEV_MEDIUM, S10_CHANGE_WEATHER);
         st = s10_replan(&g, 1, &ev, &t, &cfg, NULL, NULL, &real, &ng, &r, &info);
         CHECK((st == S10_REPLAN_OK) && (r.waypoints[0].action == HOVER), "MEDIUM + override: first waypoint is HOVER");
         CHECK(same_vec(&r.waypoints[0].position, &real), "HOVER is exactly where the drone is");
         CHECK(r.waypoints[0].hold_s == 30, "HOVER keeps its 30 s hold");
         s10_graph_free(&ng);  s10_route_free(&r);

         ev = make_event(S10_SEV_LOW, S10_CHANGE_OTHER);
         st = s10_replan(&g, 1, &ev, &t, &cfg, NULL, NULL, &real, &ng, &r, &info);
         CHECK((st == S10_REPLAN_OK) && (r.waypoints[0].action == GOTO), "LOW + override: first waypoint is the GOTO");
         CHECK(NEAR(r.waypoints[0].position.x, 100.0) && NEAR(r.waypoints[0].position.y, 100.0),
               "GOTO keeps its own position (not patched)");
         s10_graph_free(&ng);  s10_route_free(&r);

         ev = make_event(S10_SEV_CRITICAL, S10_CHANGE_COMMS);
         st = s10_replan(&g, 1, &ev, &t, &cfg, NULL, NULL, &real, &ng, &r, &info);
         CHECK((st == S10_REPLAN_OK) && (r.waypoints[0].action == RTH) &&
               NEAR(r.waypoints[0].position.x, 0.0) && NEAR(r.waypoints[0].position.y, 0.0),
               "RTH still goes to home (not patched)");
         s10_graph_free(&ng);  s10_route_free(&r);
     }

     printf("\n-- replanner: failures and determinism --\n");
     {
         Route r2;
         MissionGraph ng2;
         int i;
         bool same = true;
 
         ev = make_event(S10_SEV_LOW, S10_CHANGE_OTHER);
         st = s10_replan(&g, 5, &ev, &t, &cfg, NULL, NULL, NULL, &ng, &r, &info);
         CHECK(st == S10_REPLAN_APPLY_FAILED, "current node out of range -> APPLY_FAILED");
         CHECK(info.apply_status == S10_APPLY_BAD_ARGS, "  apply status is BAD_ARGS");
         CHECK((ng.nodes == NULL) && (r.waypoints == NULL), "  outputs left empty");
 
         st = s10_replan(NULL, 1, &ev, &t, &cfg, NULL, NULL, NULL, &ng, &r, &info);
         CHECK(st == S10_REPLAN_BAD_ARGS, "NULL graph -> BAD_ARGS");
         st = s10_replan(&g, 1, NULL, &t, &cfg, NULL, NULL, NULL, &ng, &r, &info);
         CHECK(st == S10_REPLAN_BAD_ARGS, "NULL event -> BAD_ARGS");
         st = s10_replan(&g, 1, &ev, &t, &cfg, NULL, NULL, NULL, NULL, &r, &info);
         CHECK(st == S10_REPLAN_BAD_ARGS, "NULL out graph -> BAD_ARGS");
         st = s10_replan(&g, 1, &ev, &t, &cfg, NULL, NULL, NULL, &ng, &r, NULL);
         CHECK(st == S10_REPLAN_OK, "NULL info is allowed");
         s10_graph_free(&ng);  s10_route_free(&r);
 
         CHECK(g.node_count == 3, "old graph untouched by all of the above");
 
         ev = make_event(S10_SEV_MEDIUM, S10_CHANGE_WEATHER);
         s10_replan(&g, 1, &ev, &t, &cfg, NULL, NULL, NULL, &ng, &r, &info);
         s10_replan(&g, 1, &ev, &t, &cfg, NULL, NULL, NULL, &ng2, &r2, &info);
 
         if(r.waypoint_count != r2.waypoint_count) { same = false; }
         for(i = 0; same && (i < r.waypoint_count); i++)
         {
             same = same_vec(&r.waypoints[i].position, &r2.waypoints[i].position) &&
                    NEAR(r.waypoints[i].speed_mps, r2.waypoints[i].speed_mps) &&
                    (r.waypoints[i].source_node_id == r2.waypoints[i].source_node_id);
         }
         CHECK(same && NEAR(r.cost.total, r2.cost.total), "same inputs give the same route and cost");
         s10_graph_free(&ng);   s10_route_free(&r);
         s10_graph_free(&ng2);  s10_route_free(&r2);
     }
 
     s10_graph_free(&g);
 
     printf("\n==================\nPASS: %d  FAIL: %d\n==================\n",
            g_pass, g_fail);
 
     return ((g_fail == 0) ? 0 : 1);
 }