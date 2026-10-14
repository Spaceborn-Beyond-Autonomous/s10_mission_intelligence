/**
 * @file    route_optimizer.c
 * @author  Aya Ashraf
 * @brief   Implements candidate generation, scoring and selection.
 *
 * @details
 * Candidate index = profile_index * 2 + variant, variant 0 = plan through
 * GPS-denied zones, variant 1 = plan with those zones treated as no-fly.
 * Strict "<" when comparing totals means the lowest index wins a tie.
 */

 #include "route_optimizer.h"

 #include "route_planner.h"
 
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 
 /************************************* Private Function Prototypes ************************************* */
 
 static bool copy_route_scaled(const Route *src, double factor, Route *dst);
 
 /************************************* Public Function Definitions ************************************* */
 
 bool s10_optimize_route(const MissionGraph *g,
                         const ScenarioTable *t,
                         const ObjectiveConfig *cfg,
                         S10LegRiskFn risk_fn,
                         const void *risk_ctx,
                         const S10Vec3 *start,
                         Route *best,
                         S10OptSummary *summary)
 {
     /* Whole mission: no resume node, the caller's start is required. */
     return (s10_optimize_route_from(g, t, cfg, risk_fn, risk_ctx, -1, start,
                                     best, summary, NULL));
 }
 
 
 bool s10_optimize_route_from(const MissionGraph *g,
                              const ScenarioTable *t,
                              const ObjectiveConfig *cfg,
                              S10LegRiskFn risk_fn,
                              const void *risk_ctx,
                              int resume_node_id,
                              const S10Vec3 *start_override,
                              Route *best,
                              S10OptSummary *summary,
                              S10Vec3 *start_used)
 {
     Route         base[2];            /* [0] direct, [1] safe */
     S10Vec3       base_start[2];      /* start each baseline was planned from */
     bool          base_ok[2] = { false, false };
     ScenarioTable safe_table;
     S10OptSummary local;
     S10OptSummary *s = (summary != NULL) ? summary : &local;
     double        best_total = 0.0;
     int           pi;
     int           v;
     int           i;
 
     memset(&local, 0, sizeof(local));
     memset(s, 0, sizeof(*s));
     s->best_index = -1;
 
     for(i = 0; i < S10_MAX_CANDIDATES; i++)
     {
         s->totals[i] = -1.0;
     }
 
     if(best != NULL)
     {
         memset(best, 0, sizeof(*best));
     }
 
     if(start_used != NULL)
     {
         memset(start_used, 0, sizeof(*start_used));
     }
 
     if((g == NULL) || (t == NULL) || (cfg == NULL) || (best == NULL) ||
        ((resume_node_id < 0) && (start_override == NULL)) ||
        !s10_objectives_validate(cfg))
     {
         return (false);
     }
 
     memset(base, 0, sizeof(base));
     memset(base_start, 0, sizeof(base_start));
 
     /* Variant 0: GPS-denied zones are ignored by the planner. */
     s->variant_plan[0] = s10_plan_route_from(g, t, resume_node_id,
                                              start_override, &base[0],
                                              &base_start[0]);
     base_ok[0] = (s->variant_plan[0].status == S10_PLAN_OK);
 
     /* Variant 1: GPS-denied zones become no-fly zones on a private copy. */
     safe_table = *t;
 
     if((t->no_fly_count + t->gps_denied_count) <= S10_MAX_ZONES)
     {
         for(i = 0; i < t->gps_denied_count; i++)
         {
             safe_table.no_fly[safe_table.no_fly_count++] = t->gps_denied[i];
         }
 
         s->variant_plan[1] = s10_plan_route_from(g, &safe_table,
                                                  resume_node_id,
                                                  start_override, &base[1],
                                                  &base_start[1]);
         base_ok[1] = (s->variant_plan[1].status == S10_PLAN_OK);
     }
     else
     {
         s->variant_plan[1].status = S10_PLAN_BAD_ARGUMENT;
         s->variant_plan[1].node_id = -1;
         snprintf(s->variant_plan[1].detail, sizeof(s->variant_plan[1].detail),
                  "%s", "zone table full");
     }
 
     s->candidates_tried = cfg->profile_count * 2;
 
     for(pi = 0; pi < cfg->profile_count; pi++)
     {
         for(v = 0; v < 2; v++)
         {
             int           idx = (pi * 2) + v;
             Route         cand;
             CostBreakdown cost;
 
             if(!base_ok[v])
             {
                 continue;
             }
 
             if(!copy_route_scaled(&base[v], cfg->profiles[pi].factor, &cand))
             {
                 continue;
             }
 
             if(!s10_cost_evaluate(&cand, &base_start[v], cfg, risk_fn,
                                   risk_ctx, &cost))
             {
                 s10_route_free(&cand);
                 continue;
             }
 
             cand.cost = cost;
             s->totals[idx] = cost.total;
             s->candidates_feasible++;
 
             if((s->best_index < 0) || (cost.total < best_total))
             {
                 s10_route_free(best);
                 *best = cand;
                 best_total = cost.total;
                 s->best_index = idx;
             }
             else
             {
                 s10_route_free(&cand);
             }
         }
     }
 
     s10_route_free(&base[0]);
     s10_route_free(&base[1]);
 
     if((s->best_index >= 0) && (start_used != NULL))
     {
         *start_used = base_start[s->best_index % 2];
     }
 
     return (s->best_index >= 0);
 }
 
 /************************************* Private Function Definitions ************************************* */
 
 /* Deep copy of a route with every waypoint speed multiplied by factor. */
 static bool copy_route_scaled(const Route *src, double factor, Route *dst)
 {
     int i;
 
     memset(dst, 0, sizeof(*dst));
 
     if((src->waypoints == NULL) || (src->waypoint_count <= 0))
     {
         return (false);
     }
 
     dst->waypoints = (Waypoint *)calloc((size_t)src->waypoint_count,
                                         sizeof(Waypoint));
     if(dst->waypoints == NULL)
     {
         return (false);
     }
 
     memcpy(dst->waypoints, src->waypoints,
            (size_t)src->waypoint_count * sizeof(Waypoint));
 
     for(i = 0; i < src->waypoint_count; i++)
     {
         dst->waypoints[i].speed_mps *= factor;
     }
 
     dst->waypoint_count = src->waypoint_count;
     dst->feasible = src->feasible;
 
     return (true);
 }