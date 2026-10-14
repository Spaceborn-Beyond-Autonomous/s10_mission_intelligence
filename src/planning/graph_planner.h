/**
 * @file    graph_planner.h
 * @author  Aya Ashraf
 * @brief   Path planning for a single leg: go around no-fly zones.
 *
 * @details
 * Owner: Member 3. This is the path-planning piece (one straight leg at
 * a time); mission-level ordering lives in route_planner.
 *
 * A leg is one segment between two points. If the straight line stays
 * clear of every no-fly rectangle, no extra points are needed. If it
 * crosses one, the planner searches a small graph made of the start,
 * the goal and the corners of every no-fly rectangle inflated by
 * defaults.detour_margin_m, and returns the shortest clear path as
 * intermediate points.
 *
 * No-fly zones are full height: they cannot be flown over, only around.
 * A start or goal inside a no-fly zone cannot be fixed by a detour and
 * is reported as blocked.
 */

 #ifndef S10_GRAPH_PLANNER_H
 #define S10_GRAPH_PLANNER_H
 
 /************************************* Include Part ************************************* */
 
 #include <stdbool.h>
 
 #include "s10_waypoint.h"
 #include "waypoint_generator.h"   /* ScenarioTable */
 
 /************************************* Macros Part ************************************* */
 
 #define S10_MAX_DETOUR_POINTS   8
 
 /************************************* User Data Types Part ************************************* */
 
 /**
  * @enum S10LegStatus
  * @brief Outcome of planning one leg.
 */
 typedef enum
 {
     S10_LEG_DIRECT = 0,   /* straight line is clear, no extra points */
     S10_LEG_DETOURED,     /* clear path found, points returned */
     S10_LEG_BLOCKED       /* start/goal inside a no-fly zone, or no path */
 
 } S10LegStatus;
 
 
 /**
  * @struct S10Detour
  * @brief Intermediate points of a planned leg.
 */
 typedef struct
 {
     /** @brief Detour points in flight order; excludes start and goal. */
     S10Vec3 pts[S10_MAX_DETOUR_POINTS];
 
     /** @brief Number of valid entries in pts (0 for a direct leg). */
     int count;
 
 } S10Detour;
 
 /************************************* Function Prototypes Part ************************************* */
 
 /**
  * @brief Tells whether a point lies inside any no-fly rectangle.
  * @param[in] t
  * Scenario holding the no-fly zones.
  * @param[in] x
  * Metres east of home.
  * @param[in] y
  * Metres north of home.
  * @return
  * true if the point is inside (or on the border of) a no-fly zone.
 */
 bool s10_point_in_no_fly(const ScenarioTable *t, double x, double y);
 
 
 /**
  * @brief Plans one leg from one point to another.
  * @details
  * Only x and y are used for obstacle checks; the z of every detour
  * point is interpolated between the start and goal.
  * @param[in] from
  * Leg start.
  * @param[in] to
  * Leg goal.
  * @param[in] t
  * Scenario holding the no-fly zones and detour margin.
  * @param[out] out
  * Detour points. Zeroed (count = 0) unless the status is
  * S10_LEG_DETOURED.
  * @return
  * S10_LEG_DIRECT, S10_LEG_DETOURED or S10_LEG_BLOCKED. Invalid
  * arguments are reported as S10_LEG_BLOCKED.
 */
 S10LegStatus s10_plan_leg(const S10Vec3 *from, const S10Vec3 *to,
                           const ScenarioTable *t, S10Detour *out);
 
 /************************************* End of File ************************************* */
 
 #endif /* S10_GRAPH_PLANNER_H */