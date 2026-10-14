/**
 * @file    cost_model.c
 * @author  Aya Ashraf
 * @brief   Implements route scoring on fuel, time and risk.
 */

 #include "cost_model.h"

 #include <math.h>
 #include <string.h>
 
 /************************************* Private Function Prototypes ************************************* */
 
 static double dist3d(const S10Vec3 *a, const S10Vec3 *b);
 
 static double length_inside_rect(const S10Vec3 *from, const S10Vec3 *to,
                                  const S10Rect *r);
 
 /************************************* Public Function Definitions ************************************* */
 
 double s10_leg_risk_stub(const S10Vec3 *from, const S10Vec3 *to,
                          double speed_mps, const void *ctx)
 {
     const S10RiskStubCtx *c = (const S10RiskStubCtx *)ctx;
     double inside = 0.0;
     int    i;
 
     if((from == NULL) || (to == NULL) || (c == NULL) || (c->table == NULL) ||
        (c->cfg == NULL) || (speed_mps <= 0.0))
     {
         return (0.0);
     }
 
     for(i = 0; i < c->table->gps_denied_count; i++)
     {
         inside += length_inside_rect(from, to, &c->table->gps_denied[i].rect);
     }
 
     return ((inside / speed_mps) * c->cfg->gps_denied_risk_per_s);
 }
 
 
 bool s10_cost_evaluate(const Route *route,
                        const S10Vec3 *start,
                        const ObjectiveConfig *cfg,
                        S10LegRiskFn risk_fn,
                        const void *risk_ctx,
                        CostBreakdown *out)
 {
     const S10Vec3 *prev;
     double         fuel = 0.0;
     double         time_s = 0.0;
     double         risk = 0.0;
     int            i;
 
     if(out != NULL)
     {
         memset(out, 0, sizeof(*out));
     }
 
     if((route == NULL) || (start == NULL) || (cfg == NULL) || (out == NULL) ||
        (route->waypoints == NULL) || (route->waypoint_count <= 0) ||
        (route->feasible == 0) || !s10_objectives_validate(cfg))
     {
         return (false);
     }
 
     prev = start;
 
     for(i = 0; i < route->waypoint_count; i++)
     {
         const Waypoint *w = &route->waypoints[i];
         double          d;
         double          leg_risk;
 
         /* Time needs a positive speed; anything else is a broken route. */
         if((w->speed_mps <= 0.0) || (w->hold_s < 0))
         {
             memset(out, 0, sizeof(*out));
             return (false);
         }
 
         d = dist3d(prev, &w->position);
 
         time_s += (d / w->speed_mps) + (double)w->hold_s;
 
         fuel += d * cfg->fuel_per_m *
                 pow(w->speed_mps / cfg->fuel_ref_speed_mps,
                     cfg->fuel_speed_exponent);
         fuel += (double)w->hold_s * cfg->fuel_per_hover_s;
 
         if(risk_fn != NULL)
         {
             leg_risk = risk_fn(prev, &w->position, w->speed_mps, risk_ctx);
 
             if(leg_risk > 0.0)
             {
                 risk += leg_risk;
             }
         }
 
         prev = &w->position;
     }
 
     if(risk > 1.0)
     {
         risk = 1.0;
     }
 
     out->fuel = fuel;
     out->time_s = time_s;
     out->risk = risk;
     out->total = (cfg->w_fuel * (fuel / cfg->fuel_ref)) +
                  (cfg->w_time * (time_s / cfg->time_ref_s)) +
                  (cfg->w_risk * risk);
 
     return (true);
 }
 
 /************************************* Private Function Definitions ************************************* */
 
 static double dist3d(const S10Vec3 *a, const S10Vec3 *b)
 {
     double dx = b->x - a->x;
     double dy = b->y - a->y;
     double dz = b->z - a->z;
 
     return (sqrt((dx * dx) + (dy * dy) + (dz * dz)));
 }
 
 
 /*
  * Horizontal length of the segment that lies inside the rectangle
  * (slab clipping, same idea as graph_planner).
  */
 static double length_inside_rect(const S10Vec3 *from, const S10Vec3 *to,
                                  const S10Rect *r)
 {
     double t0 = 0.0;
     double t1 = 1.0;
     double dx = to->x - from->x;
     double dy = to->y - from->y;
     double p[4];
     double q[4];
     int    i;
 
     p[0] = -dx;  q[0] = from->x - r->x_min;
     p[1] =  dx;  q[1] = r->x_max - from->x;
     p[2] = -dy;  q[2] = from->y - r->y_min;
     p[3] =  dy;  q[3] = r->y_max - from->y;
 
     for(i = 0; i < 4; i++)
     {
         if(p[i] == 0.0)
         {
             if(q[i] < 0.0)
             {
                 return (0.0);
             }
         }
         else
         {
             double t = q[i] / p[i];
 
             if(p[i] < 0.0)
             {
                 if(t > t1) { return (0.0); }
                 if(t > t0) { t0 = t; }
             }
             else
             {
                 if(t < t0) { return (0.0); }
                 if(t < t1) { t1 = t; }
             }
         }
     }
 
     return ((t1 - t0) * hypot(dx, dy));
 }