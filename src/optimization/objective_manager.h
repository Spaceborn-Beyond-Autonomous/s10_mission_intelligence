/**
 * @file    objective_manager.h
 * @author  Aya Ashraf
 * @brief   Fuel / time / risk objective configuration for S10 planning.
 *
 * @details
 * Owner: Member 3. Holds everything the optimizer and cost model need
 * that is configurable, loaded from config/objectives.yaml:
 *  - the weights of the three objectives,
 *  - reference scales that make fuel, time and risk comparable,
 *  - the physical constants of the simple cost model,
 *  - the speed profiles the optimizer tries.
 *
 * Why reference scales: fuel is in energy units, time in seconds and
 * risk is 0..1, so a raw weighted sum would be dominated by whichever
 * number is largest. The optimizer scores a route as:
 *
 *   total = w_fuel * (fuel / fuel_ref)
 *         + w_time * (time_s / time_ref_s)
 *         + w_risk * risk
 */

 #ifndef S10_OBJECTIVE_MANAGER_H
 #define S10_OBJECTIVE_MANAGER_H

 /************************************* Include Part ************************************* */

 #include <stdbool.h>

 /************************************* Macros Part ************************************* */

 #define S10_MAX_SPEED_PROFILES   4
 #define S10_PROFILE_NAME_LEN     16

 /************************************* User Data Types Part ************************************* */

 /**
  * @struct S10SpeedProfile
  * @brief A named speed multiplier tried by the optimizer.
 */
 typedef struct
 {
     /** @brief Profile name (e.g. "eco", "cruise", "fast"). */
     char name[S10_PROFILE_NAME_LEN];

     /** @brief Multiplier applied to every waypoint speed, > 0. */
     double factor;

 } S10SpeedProfile;


 /**
  * @struct ObjectiveConfig
  * @brief Weights, reference scales and cost constants.
 */
 typedef struct
 {
     /** @brief Weight of the fuel objective, >= 0. */
     double w_fuel;

     /** @brief Weight of the time objective, >= 0. */
     double w_time;

     /** @brief Weight of the risk objective, >= 0. */
     double w_risk;

     /** @brief Fuel value that counts as 1.0 after scaling, > 0. */
     double fuel_ref;

     /** @brief Mission time (s) that counts as 1.0 after scaling, > 0. */
     double time_ref_s;

     /** @brief Fuel used per metre flown at fuel_ref_speed_mps, >= 0. */
     double fuel_per_m;

     /** @brief Speed at which fuel_per_m applies, > 0. */
     double fuel_ref_speed_mps;

     /** @brief Fuel per metre scales as (speed / fuel_ref_speed_mps)
      * raised to this power, >= 0 (2.0 is drag-like). */
     double fuel_speed_exponent;

     /** @brief Fuel used per second of holding (hover/scan/pickup), >= 0. */
     double fuel_per_hover_s;

     /** @brief Risk added per second spent in a GPS-denied zone, >= 0.
      * Placeholder model until Member 4's risk estimator is wired in. */
     double gps_denied_risk_per_s;

     /** @brief Speed profiles for the optimizer, in file order. */
     S10SpeedProfile profiles[S10_MAX_SPEED_PROFILES];

     /** @brief Number of valid entries in profiles (1 to the max). */
     int profile_count;

 } ObjectiveConfig;

 /************************************* Function Prototypes Part ************************************* */

 /**
  * @brief Sets every field to a known zero value.
  * @param[out] cfg
  * Configuration to initialize.
  * @return
  * true on success, false if cfg is NULL.
 */
 bool s10_objectives_init(ObjectiveConfig *cfg);


 /**
  * @brief Checks a configuration for usable values.
  * @details
  * Rejects negative weights, a weight sum of 0, non-positive reference
  * scales or reference speed, negative cost constants, and a profile
  * count outside 1..S10_MAX_SPEED_PROFILES or any profile factor <= 0.
  * @param[in] cfg
  * Configuration to check.
  * @return
  * true if valid, false otherwise.
 */
 bool s10_objectives_validate(const ObjectiveConfig *cfg);


 /**
  * @brief Loads config/objectives.yaml and validates the result.
  * @param[out] cfg
  * Configuration to fill. Left initialized (zeroed) on failure.
  * @param[in] path
  * Path to the YAML file.
  * @return
  * true on success, false if the file is missing, malformed or invalid.
 */
 bool s10_objectives_load(ObjectiveConfig *cfg, const char *path);

 /************************************* End of File ************************************* */

 #endif /* S10_OBJECTIVE_MANAGER_H */