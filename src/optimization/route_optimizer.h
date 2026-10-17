/**
 * @file    route_optimizer.h
 * @author  Aya Ashraf
 * @brief   Picks the best of several candidate routes.
 *
 * @details
 * Owner: Member 3. Builds a fixed, ordered list of candidate routes
 * from the same mission, scores each with the cost model, and returns
 * the one with the lowest weighted total.
 *
 * Candidates are built from two choices, giving profile_count * 2 of them:
 *  - speed profile: every waypoint speed of the baseline route is
 *    multiplied by the profile factor (ObjectiveConfig.profiles);
 *  - GPS-denied handling: the route is planned either straight through
 *    GPS-denied zones (variant 0, shorter, riskier) or with those zones
 *    treated as no-fly on a copy of the ScenarioTable (variant 1,
 *    longer, safer).
 * Candidate index = profile_index * 2 + variant, so the order is fixed.
 *
 * Reproducible by construction: the candidate list is fixed, nothing is
 * random, and a tie is resolved in favour of the lowest candidate
 * index, so the same inputs always give the same route.
 *
 * Day 4: s10_optimize_route_from() does the same for the part of the
 * mission that is still ahead of the drone (used by the replanner).
 */

 #ifndef S10_ROUTE_OPTIMIZER_H
 #define S10_ROUTE_OPTIMIZER_H

 /************************************* Include Part ************************************* */

 #include <stdbool.h>

 #include "s10_graph.h"
 #include "s10_waypoint.h"
 #include "waypoint_generator.h"
 #include "route_planner.h"
 #include "objective_manager.h"
 #include "cost_model.h"

 /************************************* Macros Part ************************************* */

 #define S10_MAX_CANDIDATES   (S10_MAX_SPEED_PROFILES * 2)

 /************************************* User Data Types Part ************************************* */

 /**
  * @struct S10OptSummary
  * @brief What the optimizer tried and which candidate won.
 */
 typedef struct
 {
     /** @brief Number of candidates generated. */
     int candidates_tried;

     /** @brief Number of candidates that were feasible and scored. */
     int candidates_feasible;

     /** @brief Index of the winning candidate, or -1 if none. */
     int best_index;

     /** @brief Total score of each candidate; valid up to
      * candidates_tried, and < 0 for an infeasible candidate. */
     double totals[S10_MAX_CANDIDATES];

     /**
      * @brief Planner result of each variant's baseline route
      * ([0] direct, [1] safe). Status S10_PLAN_OK when it planned.
      * Valid only once the arguments passed validation.
     */
     S10PlanResult variant_plan[2];

 } S10OptSummary;

 /************************************* Function Prototypes Part ************************************* */

 /**
  * @brief Plans, scores and selects the best route for a mission.
  * @param[in] g
  * Mission graph to plan.
  * @param[in] t
  * Scenario supplying coordinates, defaults and zones.
  * @param[in] cfg
  * Objective weights, scales and cost constants.
  * @param[in] risk_fn
  * Per-leg risk function; NULL means zero risk.
  * @param[in] risk_ctx
  * Context passed to risk_fn.
  * @param[in] start
  * Position the drone starts from (normally home).
  * @param[out] best
  * Winning route, with its cost filled in (free with s10_route_free).
  * Left empty if no candidate was feasible.
  * @param[out] summary
  * Optional (may be NULL): per-candidate scores and the winner.
  * @return
  * true if at least one candidate was feasible; false otherwise or on
  * invalid arguments.
 */
 bool s10_optimize_route(const MissionGraph *g,
                         const ScenarioTable *t,
                         const ObjectiveConfig *cfg,
                         S10LegRiskFn risk_fn,
                         const void *risk_ctx,
                         const S10Vec3 *start,
                         Route *best,
                         S10OptSummary *summary);


 /**
  * @brief Like s10_optimize_route(), but only for the remaining mission.
  * @details
  * Each variant is planned with s10_plan_route_from(), so the winning
  * route holds only the waypoints from resume_node_id on, and its cost
  * covers only that remainder, measured from the start position used.
  * @param[in] resume_node_id
  * First node still to be executed, or < 0 for the whole mission (then
  * start_override is required).
  * @param[in] start_override
  * Optional drone position; see s10_plan_route_from() for the rules.
  * @param[out] start_used
  * Optional (may be NULL): the start the winning route was planned and
  * costed from. Zeroed if nothing was feasible.
  * All other parameters are as in s10_optimize_route().
  * @return
  * true if at least one candidate was feasible; false otherwise or on
  * invalid arguments.
 */
 bool s10_optimize_route_from(const MissionGraph *g,
                              const ScenarioTable *t,
                              const ObjectiveConfig *cfg,
                              S10LegRiskFn risk_fn,
                              const void *risk_ctx,
                              int resume_node_id,
                              const S10Vec3 *start_override,
                              Route *best,
                              S10OptSummary *summary,
                              S10Vec3 *start_used);

 /************************************* End of File ************************************* */

 #endif /* S10_ROUTE_OPTIMIZER_H */