/**
 * @file    waypoint_generator.h
 * @author  Aya Ashraf
 * @brief   Turns a MissionGraph (text locations) into raw waypoints.
 *
 * @details
 * Owner: Member 3. This module resolves each node's text reference
 * (e.g. "zone b") to coordinates using the ScenarioTable loaded from
 * config/scenarios.yaml.
 *
 * It does NOT avoid no-fly zones and does NOT compute cost. Detours are
 * inserted afterwards by route_planner (via graph_planner), and scoring
 * is done by cost_model / route_optimizer.
 *
 * Per-command rules ("previous" = where the prior waypoint ended):
 *  - GOTO         one waypoint at cmd.position (a survey area resolves
 *                 to its entry corner x_min,y_min)
 *  - PICKUP       one waypoint at cmd.position, hold = pickup_hold_s
 *  - DROPOFF      one waypoint at cmd.position, hold = dropoff_hold_s
 *  - SCAN         lawnmower across the area named in cmd.area, lines
 *                 scan_spacing_m apart, no hold
 *  - PATROL_CMD   the route named in cmd.route_or_area, once (one lap);
 *                 an area name patrols its four corners
 *  - HOVER        previous position, hold = cmd.duration_s
 *  - TAKEOFF_CMD  previous x,y (or home), climbing to cruise altitude
 *  - LAND_CMD     cmd.position if given, else previous x,y; z = 0
 *  - RTH          home x,y at cruise altitude
 * All other altitudes use cruise_alt_m and all speeds cruise_speed_mps;
 * route_optimizer overrides these when building candidates.
 */

 #ifndef S10_WAYPOINT_GENERATOR_H
 #define S10_WAYPOINT_GENERATOR_H
 
 /************************************* Include Part ************************************* */
 
 #include <stdbool.h>
 
 #include "s10_graph.h"
 #include "s10_waypoint.h"
 
 /************************************* Macros Part ************************************* */
 
 #define S10_NAME_LEN        64
 #define S10_MAX_POINTS      32
 #define S10_MAX_AREAS       16
 #define S10_MAX_ROUTES       8
 #define S10_MAX_ROUTE_PTS   32
 #define S10_MAX_ZONES       16
 
 /************************************* User Data Types Part ************************************* */
 
 /**
  * @struct S10Rect
  * @brief Axis-aligned rectangle in the local ENU frame, in metres.
 */
 typedef struct
 {
     double x_min;
     double y_min;
     double x_max;
     double y_max;
 
 } S10Rect;
 
 
 /**
  * @struct S10NamedPoint
  * @brief A named single location (GOTO, PICKUP, DROPOFF, LAND).
 */
 typedef struct
 {
     /** @brief Lowercase location name. */
     char name[S10_NAME_LEN];
 
     /** @brief Metres east of home. */
     double x;
 
     /** @brief Metres north of home. */
     double y;
 
 } S10NamedPoint;
 
 
 /**
  * @struct S10NamedRect
  * @brief A named rectangle (survey area, no-fly zone, GPS-denied zone).
 */
 typedef struct
 {
     /** @brief Lowercase region name. */
     char name[S10_NAME_LEN];
 
     /** @brief Region bounds. */
     S10Rect rect;
 
 } S10NamedRect;
 
 
 /**
  * @struct S10NamedRoute
  * @brief A named ordered loop of points (PATROL_CMD).
 */
 typedef struct
 {
     /** @brief Lowercase route name. */
     char name[S10_NAME_LEN];
 
     /** @brief Number of valid entries in pts. */
     int point_count;
 
     /** @brief Route points in flight order. */
     struct { double x; double y; } pts[S10_MAX_ROUTE_PTS];
 
 } S10NamedRoute;
 
 
 /**
  * @struct S10Defaults
  * @brief Scenario-wide default flight parameters.
 */
 typedef struct
 {
     /** @brief Cruise altitude above home, in metres. */
     double cruise_alt_m;
 
     /** @brief Cruise speed, in m/s. */
     double cruise_speed_mps;
 
     /** @brief Hold time at a pickup, in seconds. */
     int pickup_hold_s;
 
     /** @brief Hold time at a dropoff, in seconds. */
     int dropoff_hold_s;
 
     /** @brief Distance between lawnmower scan lines, in metres. */
     double scan_spacing_m;
 
     /** @brief Clearance added around a no-fly rectangle on detours. */
     double detour_margin_m;
 
 } S10Defaults;
 
 
 /**
  * @struct ScenarioTable
  * @brief In-memory form of config/scenarios.yaml.
  * @details
  * Fixed-size arrays on purpose: no heap, nothing to free. Names are
  * stored lowercase.
 */
 typedef struct
 {
     S10Defaults   defaults;
     S10Vec3       home;
 
     S10NamedPoint points[S10_MAX_POINTS];     int point_count;
     S10NamedRect  areas[S10_MAX_AREAS];       int area_count;
     S10NamedRoute routes[S10_MAX_ROUTES];     int route_count;
 
     S10NamedRect  no_fly[S10_MAX_ZONES];      int no_fly_count;
     S10NamedRect  gps_denied[S10_MAX_ZONES];  int gps_denied_count;
 
 } ScenarioTable;
 
 
 /**
  * @enum S10LocationKind
  * @brief Which ScenarioTable array a resolved name belongs to.
 */
 typedef enum
 {
     S10_LOC_NONE = 0,
     S10_LOC_POINT,
     S10_LOC_AREA,
     S10_LOC_ROUTE
 
 } S10LocationKind;
 
 
 /**
  * @struct S10Resolved
  * @brief Result of resolving a location name.
 */
 typedef struct
 {
     /** @brief Which table the name was found in. */
     S10LocationKind kind;
 
     /** @brief Index into the matching ScenarioTable array. */
     int index;
 
 } S10Resolved;
 
 
 /**
  * @enum S10GenStatus
  * @brief Outcome of waypoint generation.
 */
 typedef enum
 {
     S10_GEN_OK = 0,
     S10_GEN_UNKNOWN_LOCATION,   /* a node's name is not in the table */
     S10_GEN_MISSING_PARAM,      /* node lacks a field its command needs */
     S10_GEN_TOO_MANY_WAYPOINTS, /* would exceed S10_MAX_WAYPOINTS */
     S10_GEN_BAD_GRAPH           /* NULL/empty graph, or not a single chain */
 
 } S10GenStatus;
 
 
 /**
  * @struct S10GenResult
  * @brief Generation status plus which node failed and why.
 */
 typedef struct
 {
     /** @brief Final status. */
     S10GenStatus status;
 
     /** @brief Offending MissionNode id, or -1 when status is OK. */
     int node_id;
 
     /** @brief Human-readable reason, e.g. the unknown name. */
     char detail[96];
 
 } S10GenResult;
 
 /************************************* Function Prototypes Part ************************************* */
 
 /**
  * @brief Loads config/scenarios.yaml into a ScenarioTable.
  * @param[in] path
  * Path to the YAML file.
  * @param[out] out
  * Table to fill. Left zeroed on error, never half-filled.
  * @return
  * true on success; false if the file is missing or malformed, or a
  * table overflowed its maximum.
 */
 bool s10_scenario_load(const char *path, ScenarioTable *out);
 
 
 /**
  * @brief Resolves a location name from a graph node.
  * @details
  * Normalizes the text (lowercase, trim, strip leading "at/the/in/to"),
  * then searches points, then areas, then routes. Never guesses a
  * default location.
  * @param[in] t
  * Scenario table to search.
  * @param[in] text
  * Raw location text from a Command.
  * @param[out] out
  * Resolution result; kind is S10_LOC_NONE on a miss.
  * @return
  * true on a hit, false on a miss.
 */
 bool s10_resolve_location(const ScenarioTable *t, const char *text,
                           S10Resolved *out);
 
 
 /**
  * @brief Generates raw waypoints for a whole MissionGraph.
  * @details
  * Walks the graph in chain order. out->cost is left zeroed and
  * out->feasible is set to 1 only when the status is S10_GEN_OK.
  * @param[in] g
  * Mission graph to expand.
  * @param[in] t
  * Scenario table supplying coordinates and defaults.
  * @param[out] out
  * Route to fill (heap-allocated; free with s10_route_free). Left
  * empty on failure.
  * @return
  * Status, plus the offending node and reason on failure.
 */
 S10GenResult s10_generate_waypoints(const MissionGraph *g,
                                     const ScenarioTable *t,
                                     Route *out);
 
 /************************************* End of File ************************************* */
 
 #endif /* S10_WAYPOINT_GENERATOR_H */