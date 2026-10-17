/**
 * @file    graph_planner.c
 * @author  Aya Ashraf
 * @brief   Implements single-leg path planning around no-fly zones.
 *
 * @details
 * Visibility-graph search: the nodes are the leg's start, its goal and
 * the corners of every no-fly rectangle inflated by detour_margin_m.
 * Two nodes are connected when the straight segment between them does
 * not touch any (un-inflated) no-fly rectangle. Dijkstra finds the
 * shortest chain; the mission graph is not involved.
 */

 #include "graph_planner.h"

 #include <math.h>
 #include <string.h>

 /************************************* Private Macros Part ************************************* */

 #define MAX_NODES   (2 + (4 * S10_MAX_ZONES))

 /************************************* Private Function Prototypes ************************************* */

 static bool segment_hits_rect(double x0, double y0, double x1, double y1,
                               const S10Rect *r);

 static bool segment_clear(const ScenarioTable *t, double x0, double y0,
                           double x1, double y1);

 static double dist2d(double x0, double y0, double x1, double y1);

 /************************************* Function Definitions ************************************* */

 bool s10_point_in_no_fly(const ScenarioTable *t, double x, double y)
 {
     int i;

     if(t == NULL)
     {
         return (false);
     }

     for(i = 0; i < t->no_fly_count; i++)
     {
         const S10Rect *r = &t->no_fly[i].rect;

         if((x >= r->x_min) && (x <= r->x_max) &&
            (y >= r->y_min) && (y <= r->y_max))
         {
             return (true);
         }
     }

     return (false);
 }


 S10LegStatus s10_plan_leg(const S10Vec3 *from, const S10Vec3 *to,
                           const ScenarioTable *t, S10Detour *out)
 {
     double nx[MAX_NODES];
     double ny[MAX_NODES];
     double best[MAX_NODES];
     int    prev[MAX_NODES];
     bool   done[MAX_NODES];
     int    n = 0;
     int    i, j, z;
     int    path[MAX_NODES];
     int    path_len = 0;
     int    cur;
     double total;
     double walked;

     if(out != NULL)
     {
         memset(out, 0, sizeof(*out));
     }

     if((from == NULL) || (to == NULL) || (t == NULL) || (out == NULL))
     {
         return (S10_LEG_BLOCKED);
     }

     /*
      * A start or goal inside a no-fly zone cannot be fixed by a detour.
      */
     if(s10_point_in_no_fly(t, from->x, from->y) ||
        s10_point_in_no_fly(t, to->x, to->y))
     {
         return (S10_LEG_BLOCKED);
     }

     if(segment_clear(t, from->x, from->y, to->x, to->y)) //no detour needed
     {
         return (S10_LEG_DIRECT);
     }

     /*
      * Build the visibility-graph nodes: start, goal, inflated corners
      * (skipping corners that fall inside another no-fly zone).
      */
     nx[n] = from->x;  ny[n] = from->y;  n++;
     nx[n] = to->x;    ny[n] = to->y;    n++;

     for(z = 0; z < t->no_fly_count; z++)
     {
         const S10Rect *r = &t->no_fly[z].rect;
         double m = t->defaults.detour_margin_m;
         double cx[4] = { r->x_min - m, r->x_max + m,
                          r->x_max + m, r->x_min - m };
         double cy[4] = { r->y_min - m, r->y_min - m,
                          r->y_max + m, r->y_max + m };

         for(i = 0; i < 4; i++)
         {
             if(!s10_point_in_no_fly(t, cx[i], cy[i]))
             {
                 nx[n] = cx[i];
                 ny[n] = cy[i];
                 n++;
             }
         }
     }

     /*
      * Dijkstra over the implicit graph; O(n^2) is fine for n <= 66.
      */
     for(i = 0; i < n; i++)
     {
         best[i] = INFINITY;
         prev[i] = -1;
         done[i] = false;
     }

     best[0] = 0.0;

     for(;;)
     {
         cur = -1;

         for(i = 0; i < n; i++)
         {
             if(!done[i] && (best[i] < INFINITY) &&
                ((cur < 0) || (best[i] < best[cur])))
             {
                 cur = i;
             }
         }

         if((cur < 0) || (cur == 1))
         {
             break;
         }

         done[cur] = true;

         for(j = 0; j < n; j++)
         {
             double d;

             if(done[j])
             {
                 continue;
             }

             if(!segment_clear(t, nx[cur], ny[cur], nx[j], ny[j]))
             {
                 continue;
             }

             d = best[cur] + dist2d(nx[cur], ny[cur], nx[j], ny[j]);

             if(d < best[j])
             {
                 best[j] = d;
                 prev[j] = cur;
             }
         }
     }

     if(best[1] == INFINITY)
     {
         return (S10_LEG_BLOCKED);
     }

     /*
      * Walk back from the goal, then emit the intermediate nodes in
      * flight order.
      */
     for(cur = 1; cur >= 0; cur = prev[cur])
     {
         path[path_len++] = cur;
     }

     if((path_len - 2) > S10_MAX_DETOUR_POINTS)
     {
         return (S10_LEG_BLOCKED);
     }

     total = best[1];
     walked = 0.0;

     for(i = path_len - 2; i >= 1; i--)
     {
         int node = path[i];
         int before = path[i + 1];
         double frac;

         walked += dist2d(nx[before], ny[before], nx[node], ny[node]);
         frac = (total > 0.0) ? (walked / total) : 0.0;

         out->pts[out->count].x = nx[node];
         out->pts[out->count].y = ny[node];
         out->pts[out->count].z = from->z + (frac * (to->z - from->z));
         out->count++;
     }

     return (S10_LEG_DETOURED);
 }

 /************************************* Private Function Definitions ************************************* */

 static double dist2d(double x0, double y0, double x1, double y1)
 {
     return (hypot(x1 - x0, y1 - y0));
 }


 /*
  * Slab-clipping test. A segment that only touches the border counts as
  * hitting, which is the conservative choice.
  */
 static bool segment_hits_rect(double x0, double y0, double x1, double y1,
                               const S10Rect *r)
 {
     double t0 = 0.0;
     double t1 = 1.0;
     double dx = x1 - x0;
     double dy = y1 - y0;
     double p[4];
     double q[4];
     int    i;

     p[0] = -dx;  q[0] = x0 - r->x_min;
     p[1] =  dx;  q[1] = r->x_max - x0;
     p[2] = -dy;  q[2] = y0 - r->y_min;
     p[3] =  dy;  q[3] = r->y_max - y0;

     for(i = 0; i < 4; i++)
     {
         if(p[i] == 0.0)
         {
             if(q[i] < 0.0)
             {
                 return (false);
             }
         }
         else
         {
             double t = q[i] / p[i];

             if(p[i] < 0.0)
             {
                 if(t > t1)
                 {
                     return (false);
                 }
                 if(t > t0)
                 {
                     t0 = t;
                 }
             }
             else
             {
                 if(t < t0)
                 {
                     return (false);
                 }
                 if(t < t1)
                 {
                     t1 = t;
                 }
             }
         }
     }

     return (true);
 }


 static bool segment_clear(const ScenarioTable *t, double x0, double y0,
                           double x1, double y1)
 {
     int i;

     for(i = 0; i < t->no_fly_count; i++)
     {
         if(segment_hits_rect(x0, y0, x1, y1, &t->no_fly[i].rect))
         {
             return (false);
         }
     }

     return (true);
 }