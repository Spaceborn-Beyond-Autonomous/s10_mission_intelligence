/**
 * @file    cost_model.h
 * @author  Aya Ashraf
 * @brief   Scores a Route on fuel, time and risk.
 *
 * @details
 * Owner: Member 3. Pure scoring: it never changes the route and never
 * decides anything; route_optimizer compares the scores.
 *
 * Model (deliberately simple for the MVP):
 *  - distance : straight lines between consecutive waypoints, starting
 *               from the given start position.
 *  - time     : sum of distance / waypoint.speed_mps per leg, plus the
 *               hold_s of every waypoint.
 *  - fuel     : distance * fuel_per_m + total hold time * fuel_per_hover_s.
 *  - risk     : sum of the per-leg risk values from the risk function,
 *               clamped to [0, 1].
 *  - total    : weighted, scaled sum defined in objective_manager.h.
 *
 * The risk function is a plug-in. The built-in stub counts time spent
 * inside GPS-denied rectangles; when Member 4's risk estimator exists
 * it replaces the stub without any change to this module.
 */

 #ifndef S10_COST_MODEL_H
 #define S10_COST_MODEL_H
 
 /************************************* Include Part ************************************* */
 
 #include <stdbool.h>
 
 #include "s10_waypoint.h"
 #include "objective_manager.h"
 #include "waypoint_generator.h"   /* ScenarioTable */
 
 /************************************* User Data Types Part ************************************* */
 
 /**
  * @struct S10RiskStubCtx
  * @brief Context for s10_leg_risk_stub().
 */
 typedef struct
 {
     /** @brief Scenario holding the GPS-denied rectangles. */
     const ScenarioTable *table;
 
     /** @brief Objective constants (gps_denied_risk_per_s). */
     const ObjectiveConfig *cfg;
 
 } S10RiskStubCtx;
 
 /**
  * @brief Risk contribution of one straight leg.
  * @param[in] from
  * Leg start position.
  * @param[in] to
  * Leg end position.
  * @param[in] speed_mps
  * Speed flown on the leg.
  * @param[in] ctx
  * Implementation-specific context (the stub expects a
  * const S10RiskStubCtx *).
  * @return
  * Risk added by this leg, >= 0 (route risk is the clamped sum).
 */
 typedef double (*S10LegRiskFn)(const S10Vec3 *from,
                                const S10Vec3 *to,
                                double speed_mps,
                                const void *ctx);
 
 /************************************* Function Prototypes Part ************************************* */
 
 /**
  * @brief Built-in risk stub: risk grows with time spent inside the
  *        scenario's GPS-denied rectangles.
  * @details
  * Uses ObjectiveConfig.gps_denied_risk_per_s via the context, which
  * must therefore be an S10RiskStubCtx.
  * @param[in] from
  * Leg start.
  * @param[in] to
  * Leg end.
  * @param[in] speed_mps
  * Speed on the leg; <= 0 yields 0 risk.
  * @param[in] ctx
  * Pointer to an S10RiskStubCtx.
  * @return
  * Risk added by the leg.
 */
 double s10_leg_risk_stub(const S10Vec3 *from, const S10Vec3 *to,
                          double speed_mps, const void *ctx);
 
 
 /**
  * @brief Computes the CostBreakdown of a route.
  * @param[in] route
  * Route to score (not modified).
  * @param[in] start
  * Position the drone starts from (normally home).
  * @param[in] cfg
  * Weights, scales and cost constants.
  * @param[in] risk_fn
  * Per-leg risk function; NULL means zero risk.
  * @param[in] risk_ctx
  * Context passed to risk_fn.
  * @param[out] out
  * Resulting costs, including the weighted total.
  * @return
  * true on success; false if an argument is NULL, the route is empty
  * or infeasible, or cfg is invalid.
 */
 bool s10_cost_evaluate(const Route *route,
                        const S10Vec3 *start,
                        const ObjectiveConfig *cfg,
                        S10LegRiskFn risk_fn,
                        const void *risk_ctx,
                        CostBreakdown *out);
 
 /************************************* End of File ************************************* */
 
 #endif /* S10_COST_MODEL_H */