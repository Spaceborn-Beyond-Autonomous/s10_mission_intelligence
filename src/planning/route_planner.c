/**
 * @file    route_planner.c
 * @author  Aya Ashraf
 * @brief   Implements route planning: waypoint generation plus detours.
 */

 #include "route_planner.h"

 #include "graph_planner.h"

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>

 /************************************* Private Function Prototypes ************************************* */

 static S10PlanResult plan_result(S10PlanStatus status, int node_id,
                                  const char *detail);

 static void snap_to_real_position(const MissionGraph *g, Route *r,
                                   int resume_node_id, const S10Vec3 *real);

 /************************************* Public Function Definitions ************************************* */

 S10PlanResult s10_insert_detours(Route *r, const S10Vec3 *start,
                                  const ScenarioTable *t)
 {
     S10Detour *detours;
     Waypoint  *grown;
     int        n;
     int        i;
     int        total;
     int        w;

     if((r == NULL) || (start == NULL) || (t == NULL) ||
        (r->waypoints == NULL) || (r->waypoint_count <= 0))
     {
         return (plan_result(S10_PLAN_BAD_ARGUMENT, -1, "invalid route or argument"));
     }

     n = r->waypoint_count;

     /* A waypoint inside a no-fly zone cannot be reached by any detour. */
     for(i = 0; i < n; i++)
     {
         const S10Vec3 *p = &r->waypoints[i].position;

         if(s10_point_in_no_fly(t, p->x, p->y))
         {
             r->feasible = 0;
             return (plan_result(S10_PLAN_TARGET_IN_NO_FLY,
                                 r->waypoints[i].source_node_id,
                                 "waypoint lies inside a no-fly zone"));
         }
     }

     detours = (S10Detour *)calloc((size_t)n, sizeof(S10Detour));
     if(detours == NULL)
     {
         return (plan_result(S10_PLAN_BAD_ARGUMENT, -1, "out of memory"));
     }

     total = n;

     for(i = 0; i < n; i++)
     {
         const S10Vec3 *from = (i == 0) ? start : &r->waypoints[i - 1].position;
         S10LegStatus   st = s10_plan_leg(from, &r->waypoints[i].position, t,
                                          &detours[i]);

         if(st == S10_LEG_BLOCKED)
         {
             free(detours);
             r->feasible = 0;
             return (plan_result(S10_PLAN_NO_PATH, r->waypoints[i].source_node_id,
                                 "no clear path to this waypoint"));
         }

         total += detours[i].count;
     }

     if(total > S10_MAX_WAYPOINTS)
     {
         free(detours);
         r->feasible = 0;
         return (plan_result(S10_PLAN_TOO_MANY_WAYPOINTS, -1,
                             "detours exceed S10_MAX_WAYPOINTS"));
     }

     if(total == n)
     {
         free(detours);
         return (plan_result(S10_PLAN_OK, -1, ""));
     }

     grown = (Waypoint *)calloc((size_t)total, sizeof(Waypoint));
     if(grown == NULL)
     {
         free(detours);
         return (plan_result(S10_PLAN_BAD_ARGUMENT, -1, "out of memory"));
     }

     w = 0;

     for(i = 0; i < n; i++)
     {
         int d;

         for(d = 0; d < detours[i].count; d++)
         {
             Waypoint *dw = &grown[w++];

             dw->source_node_id = S10_NO_SOURCE_NODE;
             dw->action = GOTO;
             dw->position = detours[i].pts[d];
             dw->speed_mps = r->waypoints[i].speed_mps;
             dw->hold_s = 0;
             dw->label[0] = '\0';
         }

         grown[w++] = r->waypoints[i];
     }

     free(r->waypoints);
     free(detours);

     r->waypoints = grown;
     r->waypoint_count = total;

     return (plan_result(S10_PLAN_OK, -1, ""));
 }


 S10PlanResult s10_plan_route(const MissionGraph *g, const ScenarioTable *t,
                              const S10Vec3 *start, Route *out)
 {
     /* Whole mission: no resume node, the caller's start is required. */
     return (s10_plan_route_from(g, t, -1, start, out, NULL));
 }


 S10PlanResult s10_plan_route_from(const MissionGraph *g,
                                   const ScenarioTable *t,
                                   int resume_node_id,
                                   const S10Vec3 *start_override,
                                   Route *out,
                                   S10Vec3 *start_used)
 {
     S10GenResult  gen;
     S10PlanResult res;
     S10Vec3       start;

     if(start_used != NULL)
     {
         memset(start_used, 0, sizeof(*start_used));
     }

     if((g == NULL) || (t == NULL) || (out == NULL) ||
        ((resume_node_id < 0) && (start_override == NULL)))
     {
         if(out != NULL)
         {
             memset(out, 0, sizeof(*out));
         }
         return (plan_result(S10_PLAN_BAD_ARGUMENT, -1, "NULL argument"));
     }

     /* Always the whole graph: "previous position" comes from home. */
     gen = s10_generate_waypoints(g, t, out);

     if(gen.status != S10_GEN_OK)
     {
         res = plan_result(S10_PLAN_GENERATION_FAILED, gen.node_id, gen.detail);
         if(gen.status == S10_GEN_TOO_MANY_WAYPOINTS)
         {
             res.status = S10_PLAN_TOO_MANY_WAYPOINTS;
         }
         return (res);
     }

     if(resume_node_id < 0)
     {
         start = *start_override;
     }
     else
     {
         int idx = s10_route_find_waypoint_for_node(out, resume_node_id);

         if(idx < 0)
         {
             s10_route_free(out);
             return (plan_result(S10_PLAN_RESUME_NODE_NOT_FOUND, resume_node_id,
                                 "resume node has no waypoint"));
         }

         /* Copy the position BEFORE the array is shifted. */
         if(start_override != NULL)
         {
             start = *start_override;
         }
         else if(idx > 0)
         {
             start = out->waypoints[idx - 1].position;
         }
         else
         {
             start = t->home;
         }

         if(idx > 0)
         {
             /* Drop the flown prefix; the base pointer stays valid. */
             memmove(out->waypoints, &out->waypoints[idx],
                     (size_t)(out->waypoint_count - idx) * sizeof(Waypoint));
             memset(&out->waypoints[out->waypoint_count - idx], 0,
                    (size_t)idx * sizeof(Waypoint));
             out->waypoint_count -= idx;
         }

         /* An emergency HOVER/LAND belongs where the drone really is. */
         if(start_override != NULL)
         {
             snap_to_real_position(g, out, resume_node_id, start_override);
         }
     }

     if(start_used != NULL)
     {
         *start_used = start;
     }

     res = s10_insert_detours(out, &start, t);

     if(res.status != S10_PLAN_OK)
     {
         s10_route_free(out);
     }

     return (res);
 }

 /************************************* Private Function Definitions ************************************* */

 static S10PlanResult plan_result(S10PlanStatus status, int node_id,
                                  const char *detail)
 {
     S10PlanResult r;

     memset(&r, 0, sizeof(r));
     r.status = status;
     r.node_id = node_id;
     snprintf(r.detail, sizeof(r.detail), "%s", detail);

     return (r);
 }


 /*
  * A HOVER or LAND node with no position of its own is generated at the
  * assumed position (the end of the previous node). When the real drone
  * position is known it belongs there instead. Only the first waypoint
  * of the resume node is moved, and only for these two actions; a node
  * with an explicit position keeps it. The route holds the remainder
  * only at this point, so waypoint 0 is the resume node's first
  * waypoint and no detour points exist yet.
  */
 static void snap_to_real_position(const MissionGraph *g, Route *r,
                                   int resume_node_id, const S10Vec3 *real)
 {
     Waypoint *w;
     int       i;
     bool      positionless = false;

     if((r->waypoints == NULL) || (r->waypoint_count <= 0))
     {
         return;
     }

     w = &r->waypoints[0];

     if((w->source_node_id != resume_node_id) ||
        ((w->action != HOVER) && (w->action != LAND_CMD)))
     {
         return;
     }

     for(i = 0; i < g->node_count; i++)
     {
         if(g->nodes[i].id == resume_node_id)
         {
             positionless = (g->nodes[i].cmd.position[0] == '\0');
             break;
         }
     }

     if(!positionless)
     {
         return;
     }

     w->position.x = real->x;
     w->position.y = real->y;

     if(w->action == HOVER)
     {
         w->position.z = real->z;      /* hover where it is, at its altitude */
     }
     /* LAND keeps z = 0: land on the ground below the real position. */
 }