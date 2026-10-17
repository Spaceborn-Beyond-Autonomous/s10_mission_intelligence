/**
 * @file    s10_waypoint.h
 * @author  Aya Ashraf
 * @brief   Defines the waypoint and route contract of the S10 simulator.
 *
 * @details
 * Produced by Member 3 (planning + optimization). Consumed by Member 4
 * (validation, risk, FSM, output writers) and, at final integration,
 * by Member 1 (mission_manager / S10Mission).
 *
 * Coordinate frame: local ENU, metres, origin = home/base position.
 *  - x = metres east of home
 *  - y = metres north of home
 *  - z = metres above home
 * The core never sees lat/lon; an adapter converts at the system edge.
 */

 #ifndef S10_WAYPOINT_H
 #define S10_WAYPOINT_H

 /************************************* Include Part ************************************* */

 #include "s10_graph.h"   /* PrimitiveType (via Command) */

 /************************************* Macros Part ************************************* */

 #define S10_MAX_WAYPOINTS   256

 /* Source node id of waypoints the planner synthesizes (e.g. a detour
  * point around a no-fly zone) that match no MissionNode. */
 #define S10_NO_SOURCE_NODE  (-1)

 /************************************* User Data Types Part ************************************* */

 /**
  * @struct S10Vec3
  * @brief A point in the local ENU frame, in metres.
 */
 typedef struct
 {
     /** @brief Metres east of home. */
     double x;

     /** @brief Metres north of home. */
     double y;

     /** @brief Metres above home. */
     double z;

 } S10Vec3;


 /**
  * @struct Waypoint
  * @brief One point of a flight route.
 */
 typedef struct
 {
     /**
      * @brief Id of the MissionNode this waypoint was generated from,
      * or S10_NO_SOURCE_NODE for planner-synthesized points. Several
      * waypoints can share one node (e.g. a SCAN pattern).
     */
     int source_node_id;

     /**
      * @brief Task to perform on arrival (GOTO = pass through).
     */
     PrimitiveType action;

     /**
      * @brief Target position.
     */
     S10Vec3 position;

     /**
      * @brief Target speed on the leg INTO this waypoint, in m/s.
     */
     double speed_mps;

     /**
      * @brief Time to hold here (hover, scan, pickup, dropoff) in
      * seconds. 0 = pass through without stopping.
     */
     int hold_s;

     /**
      * @brief Original location name (e.g. "zone b"), empty if the
      * waypoint was synthesized. Used for logs only.
     */
     char label[64];

 } Waypoint;


 /**
  * @struct CostBreakdown
  * @brief Cost of a whole route along the three objective axes.
 */
 typedef struct
 {
     /** @brief Energy used (normalized units, see cost_model). */
     double fuel;

     /** @brief Total mission time in seconds. */
     double time_s;

     /** @brief Risk score, 0.0 (safe) to 1.0 (worst). */
     double risk;

     /** @brief Weighted-sum score used by the optimizer. */
     double total;

 } CostBreakdown;


 /**
  * @struct Route
  * @brief An ordered flight route and its cost.
  * @details
  * The waypoint array is heap-allocated and owned by the Route. Free it
  * with s10_route_free(). Any struct embedding a Route (e.g. S10Mission)
  * must call s10_route_free() when it is reset or destroyed.
 */
 typedef struct
 {
     /** @brief Heap array of waypoints, owned by the Route. */
     Waypoint *waypoints;

     /** @brief Number of valid entries in waypoints. */
     int waypoint_count;

     /**
      * @brief Route cost; zeroed until the optimizer fills it.
      * After a replan, cost covers the remainder only.
     */
     CostBreakdown cost;

     /**
      * @brief 1 if the planner produced a usable route, 0 otherwise.
      * This is NOT a safety verdict; acceptability is Member 4's job.
     */
     int feasible;

 } Route;

 /************************************* Function Prototypes Part ************************************* */

 /**
  * @brief Frees a Route's waypoints and zeroes the struct.
  * @param[in,out] r
  * Route to free. Safe to call on an already-empty Route.
 */
 void s10_route_free(Route *r);


 /**
  * @brief Finds the first waypoint generated from a given node.
  * @details
  * Used by the replanner to map "drone is at node N" to the waypoint
  * where the remaining route starts.
  * @param[in] r
  * Route to search.
  * @param[in] node_id
  * MissionNode id to look for.
  * @return
  * Index of the first matching waypoint, or -1 if none.
 */
 int s10_route_find_waypoint_for_node(const Route *r, int node_id);

 /************************************* End of File ************************************* */

 #endif /* S10_WAYPOINT_H */
