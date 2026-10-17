/**
 * @file    replanner.h
 * @author  Aya Ashraf
 * @brief   Replans the rest of a mission after a changed condition.
 *
 * @details
 * Owner: Member 3. Given the running mission graph, the next node to
 * execute and a change event, it
 *  1. adapts the graph (s10_graph_apply_change), then
 *  2. plans and optimizes the REMAINING mission from that node on.
 *
 * Contract:
 *  - current_node_id is the NEXT node to execute. The drone is assumed
 *    to be at the end of node current_node_id - 1 (at home when it is
 *    0). An inserted HOVER/LAND with no position of its own is placed
 *    at that assumed position, unless start_override is given: then it
 *    is placed at the real position (a HOVER at its altitude, a LAND
 *    on the ground below it). Nodes with an explicit position are
 *    never moved. Known limit: only the FIRST such waypoint uses the
 *    real position; any later positionless node still uses the
 *    assumed position, and a real position inside a no-fly zone makes
 *    the replan fail (TARGET_IN_NO_FLY / NO_PATH).
 *  - Outputs are a NEW graph and a NEW remainder route. The route's
 *    source_node_id values refer to the NEW graph, so the caller must
 *    replace its graph and route together (see s10_replan_replace).
 *  - The route holds only the remainder; its cost covers the
 *    remainder only.
 *  - It always replans, even when the graph comes back unchanged: a
 *    changed constraint (e.g. a new no-fly zone) arrives through the
 *    ScenarioTable, not through the event.
 *  - The caller passes the UPDATED ScenarioTable, and risk_ctx must
 *    point at that same table. Nothing here can detect a stale one.
 *  - The old graph is never modified. On any failure the outputs are
 *    left empty and nothing is leaked, so the active plan stays valid.
 */

 #ifndef S10_REPLANNER_H
 #define S10_REPLANNER_H

 /************************************* Include Part ************************************* */

 #include <stdbool.h>

 #include "s10_graph.h"
 #include "s10_waypoint.h"
 #include "mission_builder.h"      /* S10ChangeEvent, S10ApplyStatus */
 #include "route_planner.h"
 #include "route_optimizer.h"

 /************************************* User Data Types Part ************************************* */

 /**
  * @enum S10ReplanStatus
  * @brief Outcome of a replan.
 */
 typedef enum
 {
     S10_REPLAN_OK = 0,
     S10_REPLAN_BAD_ARGS,           /* NULL pointer or invalid config */
     S10_REPLAN_APPLY_FAILED,       /* graph could not be adapted */
     S10_REPLAN_PLAN_FAILED,        /* the planner rejected the new mission */
     S10_REPLAN_NO_FEASIBLE_ROUTE   /* planned, but no candidate scored */

 } S10ReplanStatus;


 /**
  * @struct S10ReplanInfo
  * @brief Details of a replan, filled on success and on failure.
 */
 typedef struct
 {
     /** @brief Result of the graph adaptation step. */
     S10ApplyStatus apply_status;

     /** @brief Planner failure (failing node and reason) when
      * the status is S10_REPLAN_PLAN_FAILED; otherwise S10_PLAN_OK. */
     S10PlanResult plan_result;

     /** @brief Per-candidate scores and the winner. */
     S10OptSummary opt;

     /** @brief Position the remainder was planned and costed from. */
     S10Vec3 start_used;

 } S10ReplanInfo;

 /************************************* Function Prototypes Part ************************************* */

 /**
  * @brief Adapts the mission to a change and replans what is left.
  * @param[in] old_graph
  * Running mission graph. Never modified. Ids equal array indices;
  * the first changed node has id current_node_id.
  * @param[in] current_node_id
  * Next node to execute. Pass the NEXT node; a node already in
  * progress restarts.
  * @param[in] ev
  * The change event.
  * @param[in] t
  * UPDATED scenario (zones, points, defaults).
  * @param[in] cfg
  * Objective weights and cost constants.
  * @param[in] risk_fn
  * Per-leg risk function; NULL means zero risk.
  * @param[in] risk_ctx
  * Context for risk_fn; must reference the same updated table.
  * @param[in] start_override
  * Optional real drone position; NULL uses the assumed one.
  * @param[out] out_graph
  * New graph (free with s10_graph_free). Empty on failure.
  * @param[out] out_route
  * New remainder route (free with s10_route_free). Empty on failure.
  * @param[out] info
  * Optional (may be NULL): details of the replan.
  * @return
  * S10_REPLAN_OK on success, otherwise the failing stage.
 */
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
                            S10ReplanInfo *info);


 /**
  * @brief Replans and, on success, replaces the active graph and route.
  * @details
  * Runs s10_replan() on the active graph, then swaps the results in.
  * The graph and the route are replaced TOGETHER: never keep one of the
  * old pair with one of the new pair.
  *
  * After S10_REPLAN_OK:
  *  - the route holds the REMAINDER only; its cost covers the
  *    remainder only;
  *  - its source_node_id values refer to the NEW graph;
  *  - current_node_id still names the resume node in the new graph
  *    (ids equal array indices).
  *
  * On any other status the active graph and route are untouched (same
  * pointers and contents) and nothing is leaked. The active route may
  * be empty (no plan yet); that is allowed.
  *
  * risk_ctx must reference the same UPDATED ScenarioTable as t.
  * @param[in,out] active_graph
  * Running graph; freed and replaced on success. Must not be NULL.
  * @param[in,out] active_route
  * Running route; freed and replaced on success. Must not be NULL.
  * @param[in] current_node_id
  * Next node to execute.
  * @param[in] ev
  * The change event.
  * @param[in] t
  * UPDATED scenario (zones, points, defaults).
  * @param[in] cfg
  * Objective weights and cost constants.
  * @param[in] risk_fn
  * Per-leg risk function; NULL means zero risk.
  * @param[in] risk_ctx
  * Context for risk_fn; must reference the same updated table.
  * @param[in] start_override
  * Optional real drone position; NULL uses the assumed one.
  * @param[out] info
  * Optional (may be NULL): details of the replan.
  * @return
  * S10_REPLAN_OK on success, otherwise the failing stage
  * (S10_REPLAN_BAD_ARGS if active_graph or active_route is NULL).
 */
 S10ReplanStatus s10_replan_replace(MissionGraph *active_graph,
                                    Route *active_route,
                                    int current_node_id,
                                    const S10ChangeEvent *ev,
                                    const ScenarioTable *t,
                                    const ObjectiveConfig *cfg,
                                    S10LegRiskFn risk_fn,
                                    const void *risk_ctx,
                                    const S10Vec3 *start_override,
                                    S10ReplanInfo *info);

 /************************************* End of File ************************************* */

 #endif /* S10_REPLANNER_H */