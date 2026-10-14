/**
 * @file    route_planner.h
 * @author  Aya Ashraf
 * @brief   Builds a complete, obstacle-free Route from a MissionGraph.
 *
 * @details
 * Owner: Member 3. Mission-level planning: it chains the waypoint
 * generator (names -> coordinates) with the graph planner (detours
 * around no-fly zones) and returns one baseline route. Choosing the
 * best of several routes is route_optimizer's job.
 *
 * Day 4: s10_plan_route_from() plans only the part of a mission that
 * is still ahead of the drone. It always generates waypoints for the
 * WHOLE graph first (HOVER/LAND/SCAN positions depend on the previous
 * waypoint, which is computed from home), then drops the already-flown
 * prefix and inserts detours from the drone's position.
 */

 #ifndef S10_ROUTE_PLANNER_H
 #define S10_ROUTE_PLANNER_H
 
 /************************************* Include Part ************************************* */
 
 #include "s10_graph.h"
 #include "s10_waypoint.h"
 #include "waypoint_generator.h"
 
 /************************************* User Data Types Part ************************************* */
 
 /**
  * @enum S10PlanStatus
  * @brief Outcome of route planning.
  * @note New values are appended at the end so existing values keep
  * their numbers.
 */
 typedef enum
 {
     S10_PLAN_OK = 0,
     S10_PLAN_GENERATION_FAILED,   /* waypoint generation failed */
     S10_PLAN_TARGET_IN_NO_FLY,    /* a waypoint lies inside a no-fly zone */
     S10_PLAN_NO_PATH,             /* a leg has no clear path */
     S10_PLAN_TOO_MANY_WAYPOINTS,  /* detours would exceed S10_MAX_WAYPOINTS */
     S10_PLAN_BAD_ARGUMENT,        /* NULL or invalid input */
     S10_PLAN_RESUME_NODE_NOT_FOUND /* resume node produced no waypoint */
 
 } S10PlanStatus;
 
 
 /**
  * @struct S10PlanResult
  * @brief Planning status plus which node failed and why.
 */
 typedef struct
 {
     /** @brief Final status. */
     S10PlanStatus status;
 
     /** @brief Offending MissionNode id, or -1 if not applicable. */
     int node_id;
 
     /** @brief Human-readable reason. */
     char detail[96];
 
 } S10PlanResult;
 
 /************************************* Function Prototypes Part ************************************* */
 
 /**
  * @brief Inserts detour waypoints into a route, in place.
  * @details
  * Plans every leg (start -> first waypoint, then each consecutive
  * pair) with s10_plan_leg(). Synthesized points get
  * source_node_id = S10_NO_SOURCE_NODE, action GOTO, hold 0 and the
  * speed of the waypoint they lead into. On failure the route is left
  * unchanged and feasible is set to 0.
  * @param[in,out] r
  * Route to extend (its waypoint array may be reallocated).
  * @param[in] start
  * Position the drone starts from.
  * @param[in] t
  * Scenario holding the no-fly zones.
  * @return
  * Status, plus the offending node and reason on failure.
 */
 S10PlanResult s10_insert_detours(Route *r, const S10Vec3 *start,
                                  const ScenarioTable *t);
 
 
 /**
  * @brief Plans a full route: generate waypoints, then insert detours.
  * @param[in] g
  * Mission graph to plan.
  * @param[in] t
  * Scenario supplying coordinates, defaults and no-fly zones.
  * @param[in] start
  * Position the drone starts from (normally home).
  * @param[out] out
  * Route to fill (heap-allocated; free with s10_route_free). Left
  * empty on failure. out->feasible is 1 only when the status is
  * S10_PLAN_OK; out->cost is left zeroed.
  * @return
  * Status, plus the offending node and reason on failure.
 */
 S10PlanResult s10_plan_route(const MissionGraph *g, const ScenarioTable *t,
                              const S10Vec3 *start, Route *out);
 
 
 /**
  * @brief Plans the remaining part of a mission, from a resume node on.
  * @details
  * Steps: generate waypoints for the whole graph; find the first
  * waypoint of resume_node_id and drop everything before it; insert
  * detours starting from the start position.
  *
  * Start position rules:
  *  - resume_node_id < 0: nothing is dropped and start_override is
  *    required (this is exactly s10_plan_route()).
  *  - resume_node_id >= 0: start_override if given, otherwise the
  *    position of the waypoint just before the resume point, or
  *    t->home if the resume point is the first waypoint.
  *
  * Real position: when start_override is given, a HOVER or LAND node
  * that has no position of its own (an inserted emergency action) is
  * placed at the real position too: a HOVER at the real x, y and
  * altitude, a LAND at the real x, y on the ground. Without
  * start_override such nodes stay at the assumed position (the end of
  * the previous node). Nodes with an explicit position are never moved.
  *
  * The returned route holds the remainder only: its first waypoint
  * belongs to resume_node_id, and its cost is left zeroed.
  * @param[in] g
  * Mission graph to plan (ids equal array indices).
  * @param[in] t
  * Scenario supplying coordinates, defaults and no-fly zones.
  * @param[in] resume_node_id
  * First node still to be executed, or < 0 to plan the whole mission.
  * @param[in] start_override
  * Optional drone position (see above).
  * @param[out] out
  * Remainder route (free with s10_route_free). Left empty on failure.
  * @param[out] start_used
  * Optional (may be NULL): the start position the detours were planned
  * from. Pass the same position to the cost model.
  * @return
  * Status, plus the offending node and reason on failure.
 */
 S10PlanResult s10_plan_route_from(const MissionGraph *g,
                                   const ScenarioTable *t,
                                   int resume_node_id,
                                   const S10Vec3 *start_override,
                                   Route *out,
                                   S10Vec3 *start_used);
 
 /************************************* End of File ************************************* */
 
 #endif /* S10_ROUTE_PLANNER_H */