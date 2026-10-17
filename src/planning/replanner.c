/**
 * @file    replanner.c
 * @author  Aya Ashraf
 * @brief   Implements replanning after a changed condition.
 */

 #include "replanner.h"

 #include <string.h>

 /************************************* Public Function Definitions ************************************* */

 S10ReplanStatus s10_replan(const MissionGraph *old_graph,
                            int current_node_id,
                            const S10ChangeEvent *ev,
                            const ScenarioTable *t,
                            const ObjectiveConfig *cfg,
                            S10LegRiskFn risk_fn,
                            const void *risk_ctx,
                            const S10Vec3 *start_override,
                            MissionGraph *out_graph,
                            Route *out_route,
                            S10ReplanInfo *info)
 {
     S10ReplanInfo  local;
     S10ReplanInfo *inf = (info != NULL) ? info : &local;
     MissionGraph   new_graph;
     Route          best;
     S10ApplyStatus apply;

     memset(&local, 0, sizeof(local));
     memset(inf, 0, sizeof(*inf));
     inf->opt.best_index = -1;

     if(out_graph != NULL)
     {
         memset(out_graph, 0, sizeof(*out_graph));
     }

     if(out_route != NULL)
     {
         memset(out_route, 0, sizeof(*out_route));
     }

     if((old_graph == NULL) || (ev == NULL) || (t == NULL) || (cfg == NULL) ||
        (out_graph == NULL) || (out_route == NULL) ||
        !s10_objectives_validate(cfg))
     {
         return (S10_REPLAN_BAD_ARGS);
     }

     /* Step 1: adapt the graph. old_graph is never modified. */
     memset(&new_graph, 0, sizeof(new_graph));
     apply = s10_graph_apply_change(old_graph, current_node_id, ev, &new_graph);
     inf->apply_status = apply;

     if((apply != S10_APPLY_CHANGED) && (apply != S10_APPLY_UNCHANGED))
     {
         s10_graph_free(&new_graph);
         return (S10_REPLAN_APPLY_FAILED);
     }

     /*
      * Step 2: plan and optimize the remainder. Always done, even for
      * UNCHANGED: a zone change reaches us through the table.
      * Ids equal indices, so current_node_id is the resume node in the
      * new graph too.
      */
     memset(&best, 0, sizeof(best));

     if(!s10_optimize_route_from(&new_graph, t, cfg, risk_fn, risk_ctx,
                                 current_node_id, start_override, &best,
                                 &inf->opt, &inf->start_used))
     {
         s10_graph_free(&new_graph);
         s10_route_free(&best);

         if(inf->opt.variant_plan[0].status != S10_PLAN_OK)
         {
             inf->plan_result = inf->opt.variant_plan[0];
             return (S10_REPLAN_PLAN_FAILED);
         }

         return (S10_REPLAN_NO_FEASIBLE_ROUTE);
     }

     /* Success: hand ownership of both results to the caller. */
     *out_graph = new_graph;
     *out_route = best;

     return (S10_REPLAN_OK);
 }


 S10ReplanStatus s10_replan_replace(MissionGraph *active_graph,
                                    Route *active_route,
                                    int current_node_id,
                                    const S10ChangeEvent *ev,
                                    const ScenarioTable *t,
                                    const ObjectiveConfig *cfg,
                                    S10LegRiskFn risk_fn,
                                    const void *risk_ctx,
                                    const S10Vec3 *start_override,
                                    S10ReplanInfo *info)
 {
     MissionGraph    new_graph;
     Route           new_route;
     S10ReplanStatus st;

     if((active_graph == NULL) || (active_route == NULL))
     {
         return (S10_REPLAN_BAD_ARGS);
     }

     memset(&new_graph, 0, sizeof(new_graph));
     memset(&new_route, 0, sizeof(new_route));

     st = s10_replan(active_graph, current_node_id, ev, t, cfg, risk_fn,
                     risk_ctx, start_override, &new_graph, &new_route, info);

     if(st != S10_REPLAN_OK)
     {
         /* s10_replan leaves its outputs empty and leaks nothing. */
         return (st);
     }

     /* Success: free the old pair, then move the new pair in (no copy). */
     s10_graph_free(active_graph);
     s10_route_free(active_route);

     *active_graph = new_graph;
     *active_route = new_route;

     return (S10_REPLAN_OK);
 }