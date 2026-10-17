/**
 * @file    waypoint_generator.c
 * @author  Aya Ashraf
 * @brief   Implements scenario loading, name resolution, waypoint
 *          generation and the Route helpers from s10_waypoint.h.
 *
 * @details
 * The YAML loader reads libyaml events and keeps a small stack of
 * frames (one per open mapping/sequence). Every section of
 * scenarios.yaml has the shape  section -> name -> field -> number,
 * except routes, whose points are a sequence of {x, y} maps.
 */

 #include "waypoint_generator.h"

 #include <ctype.h>
 #include <math.h>
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <yaml.h>

 /************************************* Private Macros Part ************************************* */

 #define YAML_MAX_DEPTH      8
 #define YAML_KEY_LEN        S10_NAME_LEN
 #define SAME_POS_EPS        1e-6

 /************************************* Private Data Types Part ************************************* */

 typedef struct
 {
     bool is_seq; //sequence or mapping
     bool expecting_key;  //for mapping
     char key[YAML_KEY_LEN];

 } YamlFrame;


 /* Growing waypoint list used while generating. */
 typedef struct
 {
     Waypoint *items;
     int       count;
     bool      overflow;

 } WpList;

 /************************************* Private Function Prototypes ************************************* */

 static void normalize_name(const char *in, char *out, size_t out_len);
 static bool parse_double(const char *text, double *value);
 static bool parse_int(const char *text, int *value);
 static bool apply_scalar(ScenarioTable *t, const YamlFrame *f, int depth,
                          const char *value);
 static bool open_entry(ScenarioTable *t, const char *section,
                        const char *name);
 static bool scenario_validate(const ScenarioTable *t);
 static bool rect_set(S10Rect *r, const char *field, double v);

 static bool wp_add(WpList *l, int node_id, PrimitiveType action, double x,
                    double y, double z, double speed, int hold_s,
                    const char *label);
 static bool resolve_xy(const ScenarioTable *t, const char *text,
                        double *x, double *y, S10Resolved *res);
 static S10GenResult gen_fail(S10GenStatus status, int node_id,
                              const char *fmt, const char *arg);

 /************************************* Public Function Definitions ************************************* */

 bool s10_scenario_load(const char *path, ScenarioTable *out)
 {
     FILE          *file;
     yaml_parser_t  parser;
     yaml_event_t   event;
     YamlFrame      frames[YAML_MAX_DEPTH];
     int            depth = 0;
     bool           parsing = true;
     bool           ok = false;
     bool           parser_ready = false;
     bool           seen_defaults = false;
     ScenarioTable *t;

     if((path == NULL) || (out == NULL))
     {
         return (false);
     }

     memset(out, 0, sizeof(*out));

     /* Parse into a heap copy so *out is never half-filled. */
     t = (ScenarioTable *)calloc(1, sizeof(*t));
     if(t == NULL)
     {
         return (false);
     }

     file = fopen(path, "r");
     if(file == NULL)
     {
         free(t);
         return (false);
     }

     if(!yaml_parser_initialize(&parser))
     {
         fclose(file);
         free(t);
         return (false);
     }

     parser_ready = true;
     yaml_parser_set_input_file(&parser, file);

     while(parsing)
     {
         if(!yaml_parser_parse(&parser, &event))
         {
             fprintf(stderr, "s10_scenario_load: %s: line %zu: %s\n", path,
                     parser.problem_mark.line + 1U,
                     (parser.problem != NULL) ? parser.problem : "parse error");
             goto cleanup;
         }

         switch(event.type)
         {
             case YAML_STREAM_END_EVENT:
             {
                 parsing = false;
                 break;
             }

             case YAML_MAPPING_START_EVENT:
             case YAML_SEQUENCE_START_EVENT:
             {
                 if(depth >= YAML_MAX_DEPTH)
                 {
                     yaml_event_delete(&event);
                     goto cleanup;
                 }

                 frames[depth].is_seq = (event.type == YAML_SEQUENCE_START_EVENT);
                 frames[depth].expecting_key = true;
                 frames[depth].key[0] = '\0';

                 /* A new {x, y} map inside a route = a new route point. */
                 if((event.type == YAML_MAPPING_START_EVENT) && (depth == 3) &&
                    frames[2].is_seq &&
                    (strcmp(frames[0].key, "routes") == 0))
                 {
                     S10NamedRoute *rt = &t->routes[t->route_count - 1];

                     if(rt->point_count >= S10_MAX_ROUTE_PTS)
                     {
                         yaml_event_delete(&event);
                         goto cleanup;
                     }

                     rt->point_count++;
                 }

                 depth++;
                 break;
             }

             case YAML_MAPPING_END_EVENT:
             case YAML_SEQUENCE_END_EVENT:
             {
                 if(depth == 0)
                 {
                     yaml_event_delete(&event);
                     goto cleanup;
                 }

                 depth--;

                 /* The parent mapping can read its next key. */
                 if((depth > 0) && !frames[depth - 1].is_seq)
                 {
                     frames[depth - 1].expecting_key = true;
                 }
                 break;
             }

             case YAML_SCALAR_EVENT:
             {
                 const char *value = (const char *)event.data.scalar.value;

                 if(depth == 0)
                 {
                     yaml_event_delete(&event);
                     goto cleanup;
                 }

                 if(frames[depth - 1].is_seq)
                 {
                     /* Bare scalars in a sequence are not part of the format. */
                     yaml_event_delete(&event);
                     goto cleanup;
                 }

                 if(frames[depth - 1].expecting_key)
                 {
                     if(event.data.scalar.length >= YAML_KEY_LEN)
                     {
                         yaml_event_delete(&event);
                         goto cleanup;
                     }

                     normalize_name(value, frames[depth - 1].key, YAML_KEY_LEN);
                     frames[depth - 1].expecting_key = false;

                     /* A name key inside a named section opens a new entry. */
                     if((depth == 2) && !open_entry(t, frames[0].key,
                                                    frames[1].key))
                     {
                         fprintf(stderr, "s10_scenario_load: %s: bad or "
                                 "duplicate entry '%s' in '%s'\n", path,
                                 frames[1].key, frames[0].key);
                         yaml_event_delete(&event);
                         goto cleanup;
                     }
                 }
                 else
                 {
                     if(!apply_scalar(t, frames, depth, value))
                     {
                         fprintf(stderr, "s10_scenario_load: %s: bad value "
                                 "'%s' for '%s'\n", path, value,
                                 frames[depth - 1].key);
                         yaml_event_delete(&event);
                         goto cleanup;
                     }

                     if(strcmp(frames[0].key, "defaults") == 0)
                     {
                         seen_defaults = true;
                     }

                     frames[depth - 1].expecting_key = true;
                 }
                 break;
             }

             default:
             {
                 break;
             }
         }

         yaml_event_delete(&event);
     }

     if(!seen_defaults || !scenario_validate(t))
     {
         fprintf(stderr, "s10_scenario_load: %s: missing or invalid "
                 "values\n", path);
         goto cleanup;
     }

     memcpy(out, t, sizeof(*out));
     ok = true;

 cleanup:

     if(parser_ready)
     {
         yaml_parser_delete(&parser);
     }

     fclose(file);
     free(t);

     return (ok);
 }


 bool s10_resolve_location(const ScenarioTable *t, const char *text,
                           S10Resolved *out)
 {
     char norm[S10_NAME_LEN];
     int  i;

     if(out != NULL)
     {
         out->kind = S10_LOC_NONE;
         out->index = -1;
     }

     if((t == NULL) || (text == NULL) || (out == NULL))
     {
         return (false);
     }

     normalize_name(text, norm, sizeof(norm));

     if(norm[0] == '\0')
     {
         return (false);
     }

     for(i = 0; i < t->point_count; i++)
     {
         if(strcmp(t->points[i].name, norm) == 0)
         {
             out->kind = S10_LOC_POINT;
             out->index = i;
             return (true);
         }
     }

     for(i = 0; i < t->area_count; i++)
     {
         if(strcmp(t->areas[i].name, norm) == 0)
         {
             out->kind = S10_LOC_AREA;
             out->index = i;
             return (true);
         }
     }

     for(i = 0; i < t->route_count; i++)
     {
         if(strcmp(t->routes[i].name, norm) == 0)
         {
             out->kind = S10_LOC_ROUTE;
             out->index = i;
             return (true);
         }
     }

     return (false);
 }


 S10GenResult s10_generate_waypoints(const MissionGraph *g,
                                     const ScenarioTable *t,
                                     Route *out)
 {
     S10GenResult res;
     WpList       list;
     int          order[S10_MAX_NODES];
     int          order_len = 0;
     double       px, py, pz;       /* where the previous waypoint ended */
     const double alt = (t != NULL) ? t->defaults.cruise_alt_m : 0.0;
     const double spd = (t != NULL) ? t->defaults.cruise_speed_mps : 0.0;
     int          k;

     memset(&res, 0, sizeof(res));
     res.node_id = -1;

     if(out != NULL)
     {
         memset(out, 0, sizeof(*out));
     }

     if((g == NULL) || (t == NULL) || (out == NULL))
     {
         return (gen_fail(S10_GEN_BAD_GRAPH, -1, "%s", "NULL argument"));
     }

     if((g->nodes == NULL) || (g->node_count <= 0) ||
        (g->node_count > S10_MAX_NODES))
     {
         return (gen_fail(S10_GEN_BAD_GRAPH, -1, "%s", "empty or oversized graph"));
     }

     /*
      * Find the chain order: the head is the only node no edge points at,
      * then follow from_id -> to_id.
      */
     {
         int head = -1;
         int i, e;

         for(i = 0; i < g->node_count; i++)
         {
             bool has_in = false;

             for(e = 0; e < g->edge_count; e++)
             {
                 if(g->edges[e].to_id == g->nodes[i].id)
                 {
                     has_in = true;
                     break;
                 }
             }

             if(!has_in)
             {
                 if(head >= 0)
                 {
                     return (gen_fail(S10_GEN_BAD_GRAPH, -1, "%s",
                                      "more than one start node"));
                 }
                 head = i;
             }
         }

         if(head < 0)
         {
             return (gen_fail(S10_GEN_BAD_GRAPH, -1, "%s", "graph has a cycle"));
         }

         order[order_len++] = head;

         while(order_len < g->node_count)
         {
             int  cur_id = g->nodes[order[order_len - 1]].id;
             int  next = -1;

             for(e = 0; e < g->edge_count; e++)
             {
                 if(g->edges[e].from_id == cur_id)
                 {
                     for(i = 0; i < g->node_count; i++)
                     {
                         if(g->nodes[i].id == g->edges[e].to_id)
                         {
                             next = i;
                             break;
                         }
                     }
                     break;
                 }
             }

             if(next < 0)
             {
                 break;
             }

             order[order_len++] = next;
         }

         if(order_len != g->node_count)
         {
             return (gen_fail(S10_GEN_BAD_GRAPH, -1, "%s",
                              "graph is not a single chain"));
         }
     }

     list.items = (Waypoint *)calloc(S10_MAX_WAYPOINTS, sizeof(Waypoint));
     list.count = 0;
     list.overflow = false;

     if(list.items == NULL)
     {
         return (gen_fail(S10_GEN_BAD_GRAPH, -1, "%s", "out of memory"));
     }

     px = t->home.x;
     py = t->home.y;
     pz = t->home.z;

     for(k = 0; k < order_len; k++)
     {
         const MissionNode *n = &g->nodes[order[k]];
         const Command     *c = &n->cmd;
         double             x = px;
         double             y = py;
         double             z = alt;
         S10Resolved        r;
         char               label[64];

         label[0] = '\0';

         switch(c->type)
         {
             case GOTO:
             case PICKUP:
             case DROPOFF:
             {
                 int hold = 0;

                 if(c->position[0] == '\0')
                 {
                     res = gen_fail(S10_GEN_MISSING_PARAM, n->id, "%s",
                                    "command has no position");
                     goto fail;
                 }

                 if(!resolve_xy(t, c->position, &x, &y, &r))
                 {
                     res = gen_fail(S10_GEN_UNKNOWN_LOCATION, n->id,
                                    "unknown location '%s'", c->position);
                     goto fail;
                 }

                 if(c->type == PICKUP)
                 {
                     hold = t->defaults.pickup_hold_s;
                 }
                 else if(c->type == DROPOFF)
                 {
                     hold = t->defaults.dropoff_hold_s;
                 }

                 normalize_name(c->position, label, sizeof(label));
                 wp_add(&list, n->id, c->type, x, y, z, spd, hold, label);
                 px = x; py = y; pz = z;
                 break;
             }

             case HOVER:
             case TAKEOFF_CMD:
             case LAND_CMD:
             {
                 int hold = 0;

                 if(c->position[0] != '\0')
                 {
                     if(!resolve_xy(t, c->position, &x, &y, &r))
                     {
                         res = gen_fail(S10_GEN_UNKNOWN_LOCATION, n->id,
                                        "unknown location '%s'", c->position);
                         goto fail;
                     }
                     normalize_name(c->position, label, sizeof(label));
                 }

                 if(c->type == HOVER)
                 {
                     hold = (c->duration_s > 0) ? c->duration_s : 0;
                 }
                 else if(c->type == LAND_CMD)
                 {
                     z = 0.0;
                 }

                 wp_add(&list, n->id, c->type, x, y, z, spd, hold, label);
                 px = x; py = y; pz = z;
                 break;
             }

             case RTH:
             {
                 wp_add(&list, n->id, RTH, t->home.x, t->home.y, alt, spd, 0,
                        "home");
                 px = t->home.x; py = t->home.y; pz = alt;
                 break;
             }

             case SCAN:
             {
                 const S10Rect *a;
                 int            row, rows;
                 double         ycur;

                 if(c->area[0] == '\0')
                 {
                     res = gen_fail(S10_GEN_MISSING_PARAM, n->id, "%s",
                                    "SCAN has no area");
                     goto fail;
                 }

                 if(!s10_resolve_location(t, c->area, &r) ||
                    (r.kind != S10_LOC_AREA))
                 {
                     res = gen_fail(S10_GEN_UNKNOWN_LOCATION, n->id,
                                    "'%s' is not a survey area", c->area);
                     goto fail;
                 }

                 a = &t->areas[r.index].rect;
                 normalize_name(c->area, label, sizeof(label));

                 /* Lines every scan_spacing_m, plus a last line on the far edge. */
                 rows = (int)floor((a->y_max - a->y_min) /
                                   t->defaults.scan_spacing_m) + 1;
                 if(((a->y_min + ((rows - 1) * t->defaults.scan_spacing_m)) <
                     a->y_max) )
                 {
                     rows++;
                 }

                 for(row = 0; row < rows; row++)
                 {
                     double xa = ((row % 2) == 0) ? a->x_min : a->x_max;
                     double xb = ((row % 2) == 0) ? a->x_max : a->x_min;

                     ycur = a->y_min + (row * t->defaults.scan_spacing_m);
                     if(ycur > a->y_max)
                     {
                         ycur = a->y_max;
                     }

                     /* Skip a first point we are already standing on. */
                     if(!((row == 0) && (fabs(px - xa) < SAME_POS_EPS) &&
                          (fabs(py - ycur) < SAME_POS_EPS)))
                     {
                         wp_add(&list, n->id, SCAN, xa, ycur, alt, spd, 0, label);
                     }

                     wp_add(&list, n->id, SCAN, xb, ycur, alt, spd, 0, label);
                     px = xb; py = ycur; pz = alt;
                 }
                 break;
             }

             case PATROL_CMD:
             {
                 int p;

                 if(c->route_or_area[0] == '\0')
                 {
                     res = gen_fail(S10_GEN_MISSING_PARAM, n->id, "%s",
                                    "PATROL has no route");
                     goto fail;
                 }

                 if(!s10_resolve_location(t, c->route_or_area, &r) ||
                    (r.kind == S10_LOC_POINT))
                 {
                     res = gen_fail(S10_GEN_UNKNOWN_LOCATION, n->id,
                                    "'%s' is not a route or area",
                                    c->route_or_area);
                     goto fail;
                 }

                 normalize_name(c->route_or_area, label, sizeof(label));

                 if(r.kind == S10_LOC_ROUTE)
                 {
                     const S10NamedRoute *rt = &t->routes[r.index];

                     for(p = 0; p < rt->point_count; p++)
                     {
                         wp_add(&list, n->id, PATROL_CMD, rt->pts[p].x,
                                rt->pts[p].y, alt, spd, 0, label);
                         px = rt->pts[p].x; py = rt->pts[p].y; pz = alt;
                     }
                 }
                 else
                 {
                     const S10Rect *a = &t->areas[r.index].rect;
                     double cx[4] = { a->x_min, a->x_max, a->x_max, a->x_min };
                     double cy[4] = { a->y_min, a->y_min, a->y_max, a->y_max };

                     for(p = 0; p < 4; p++)
                     {
                         wp_add(&list, n->id, PATROL_CMD, cx[p], cy[p], alt,
                                spd, 0, label);
                         px = cx[p]; py = cy[p]; pz = alt;
                     }
                 }
                 break;
             }

             default:
             {
                 res = gen_fail(S10_GEN_MISSING_PARAM, n->id, "%s",
                                "unknown command type");
                 goto fail;
             }
         }

         if(list.overflow)
         {
             res = gen_fail(S10_GEN_TOO_MANY_WAYPOINTS, n->id, "%s",
                            "route exceeds S10_MAX_WAYPOINTS");
             goto fail;
         }
     }

     (void)pz;

     out->waypoints = list.items;
     out->waypoint_count = list.count;
     out->feasible = 1;
     res.status = S10_GEN_OK;
     res.node_id = -1;
     res.detail[0] = '\0';

     return (res);

 fail:

     free(list.items);
     memset(out, 0, sizeof(*out));

     return (res);
 }

 /*
  * Route helpers declared in s10_waypoint.h. They live here because this
  * module is what creates Routes.
  */
 void s10_route_free(Route *r)
 {
     if(r == NULL)
     {
         return;
     }

     free(r->waypoints);

     memset(r, 0, sizeof(*r));
 }


 int s10_route_find_waypoint_for_node(const Route *r, int node_id)
 {
     int i;

     /*
      * The sentinel marks planner-synthesized points; it is not a node,
      * so it never matches.
      */
     if((r == NULL) || (r->waypoints == NULL) ||
        (node_id == S10_NO_SOURCE_NODE))
     {
         return (-1);
     }

     for(i = 0; i < r->waypoint_count; i++)
     {
         if(r->waypoints[i].source_node_id == node_id)
         {
             return (i);
         }
     }

     return (-1);
 }


 /************************************* Private Function Definitions ************************************* */

 /* Lowercase, trim, collapse spaces, strip leading "at/the/in/to". */
 static void normalize_name(const char *in, char *out, size_t out_len)
 {
     char   tmp[S10_NAME_LEN * 2];
     size_t n = 0;
     size_t i;
     bool   prev_space = true;
     char  *p;
     bool   again = true;

     for(i = 0; (in[i] != '\0') && (n < (sizeof(tmp) - 1U)); i++)
     {
         unsigned char ch = (unsigned char)in[i];

         if(isspace(ch))
         {
             if(!prev_space)
             {
                 tmp[n++] = ' ';
             }
             prev_space = true;
         }
         else
         {
             tmp[n++] = (char)tolower(ch);
             prev_space = false;
         }
     }

     while((n > 0U) && (tmp[n - 1U] == ' '))
     {
         n--;
     }
     tmp[n] = '\0';

     p = tmp;

     while(again)
     {
         static const char *fillers[] = { "at ", "the ", "in ", "to " };
         size_t f;

         again = false;

         for(f = 0; f < (sizeof(fillers) / sizeof(fillers[0])); f++)
         {
             size_t len = strlen(fillers[f]);

             if(strncmp(p, fillers[f], len) == 0)
             {
                 p += len;
                 again = true;
             }
         }
     }

     /*
      * A name that does not fit is returned empty, never truncated: a
      * truncated name could silently match a different location.
      */
     if(strlen(p) >= out_len)
     {
         out[0] = '\0';
         return;
     }

     memcpy(out, p, strlen(p) + 1U);
 }


 static bool parse_double(const char *text, double *value)
 {
     char  *end;
     double v = strtod(text, &end);

     if((end == text) || (*end != '\0') || !isfinite(v))
     {
         return (false);
     }

     *value = v;
     return (true);
 }


 static bool parse_int(const char *text, int *value)
 {
     char *end;
     long  v = strtol(text, &end, 10);

     if((end == text) || (*end != '\0') || (v < 0) || (v > 1000000L))
     {
         return (false);
     }

     *value = (int)v;
     return (true);
 }


 static bool rect_set(S10Rect *r, const char *field, double v)
 {
     if(strcmp(field, "x_min") == 0)      { r->x_min = v; }
     else if(strcmp(field, "x_max") == 0) { r->x_max = v; }
     else if(strcmp(field, "y_min") == 0) { r->y_min = v; }
     else if(strcmp(field, "y_max") == 0) { r->y_max = v; }
     else                                 { return (false); }

     return (true);
 }


 /* Creates the next entry of a named section; rejects overflow/duplicates. */
 static bool open_entry(ScenarioTable *t, const char *section,
                        const char *name)
 {
     int i;

     if(name[0] == '\0')
     {
         return (false);
     }

 #define OPEN_NAMED(arr, cnt, max)                                  \
     do {                                                           \
         for(i = 0; i < (cnt); i++)                                 \
         {                                                          \
             if(strcmp((arr)[i].name, name) == 0) { return (false); } \
         }                                                          \
         if((cnt) >= (max)) { return (false); }                     \
         snprintf((arr)[(cnt)].name, S10_NAME_LEN, "%s", name);     \
         (cnt)++;                                                   \
         return (true);                                             \
     } while(0)

     if(strcmp(section, "points") == 0)
     {
         OPEN_NAMED(t->points, t->point_count, S10_MAX_POINTS);
     }
     if(strcmp(section, "areas") == 0)
     {
         OPEN_NAMED(t->areas, t->area_count, S10_MAX_AREAS);
     }
     if(strcmp(section, "routes") == 0)
     {
         OPEN_NAMED(t->routes, t->route_count, S10_MAX_ROUTES);
     }
     if(strcmp(section, "no_fly") == 0)
     {
         OPEN_NAMED(t->no_fly, t->no_fly_count, S10_MAX_ZONES);
     }
     if(strcmp(section, "gps_denied") == 0)
     {
         OPEN_NAMED(t->gps_denied, t->gps_denied_count, S10_MAX_ZONES);
     }

 #undef OPEN_NAMED

     /* A depth-2 key in a non-named section (home, defaults) is a field. */
     return (true);
 }


 /* Stores one scalar value at the position described by the frame stack. */
 static bool apply_scalar(ScenarioTable *t, const YamlFrame *f, int depth,
                          const char *value)
 {
     const char *section = f[0].key;
     double      v;

     if(depth == 1)
     {
         /* Top-level scalar: only "frame" is allowed, and only local_enu_m. */
         return ((strcmp(section, "frame") == 0) &&
                 (strcmp(value, "local_enu_m") == 0));
     }

     if((strcmp(section, "home") == 0) && (depth == 2))
     {
         if(!parse_double(value, &v)) { return (false); }
         if(strcmp(f[1].key, "x") == 0)      { t->home.x = v; }
         else if(strcmp(f[1].key, "y") == 0) { t->home.y = v; }
         else if(strcmp(f[1].key, "z") == 0) { t->home.z = v; }
         else                                { return (false); }
         return (true);
     }

     if((strcmp(section, "defaults") == 0) && (depth == 2))
     {
         S10Defaults *d = &t->defaults;
         const char  *k = f[1].key;
         int          iv;

         if(strcmp(k, "pickup_hold_s") == 0)
         {
             if(!parse_int(value, &iv)) { return (false); }
             d->pickup_hold_s = iv;
             return (true);
         }
         if(strcmp(k, "dropoff_hold_s") == 0)
         {
             if(!parse_int(value, &iv)) { return (false); }
             d->dropoff_hold_s = iv;
             return (true);
         }

         if(!parse_double(value, &v)) { return (false); }
         if(strcmp(k, "cruise_alt_m") == 0)          { d->cruise_alt_m = v; }
         else if(strcmp(k, "cruise_speed_mps") == 0) { d->cruise_speed_mps = v; }
         else if(strcmp(k, "scan_spacing_m") == 0)   { d->scan_spacing_m = v; }
         else if(strcmp(k, "detour_margin_m") == 0)  { d->detour_margin_m = v; }
         else                                        { return (false); }
         return (true);
     }

     if(depth == 3)
     {
         const char *field = f[2].key;

         if(!parse_double(value, &v)) { return (false); }

         if(strcmp(section, "points") == 0)
         {
             S10NamedPoint *p = &t->points[t->point_count - 1];

             if(strcmp(field, "x") == 0)      { p->x = v; return (true); }
             if(strcmp(field, "y") == 0)      { p->y = v; return (true); }
             return (false);
         }
         if(strcmp(section, "areas") == 0)
         {
             return (rect_set(&t->areas[t->area_count - 1].rect, field, v));
         }
         if(strcmp(section, "no_fly") == 0)
         {
             return (rect_set(&t->no_fly[t->no_fly_count - 1].rect, field, v));
         }
         if(strcmp(section, "gps_denied") == 0)
         {
             return (rect_set(&t->gps_denied[t->gps_denied_count - 1].rect,
                              field, v));
         }
         return (false);
     }

     /* routes -> name -> [ {x, y} ]  : depth 4, inside a sequence. */
     if((strcmp(section, "routes") == 0) && (depth == 4) && f[2].is_seq)
     {
         S10NamedRoute *rt = &t->routes[t->route_count - 1];
         int            idx = rt->point_count - 1;

         if((idx < 0) || !parse_double(value, &v)) { return (false); }
         if(strcmp(f[3].key, "x") == 0)      { rt->pts[idx].x = v; return (true); }
         if(strcmp(f[3].key, "y") == 0)      { rt->pts[idx].y = v; return (true); }
         return (false);
     }

     return (false);
 }


 static bool scenario_validate(const ScenarioTable *t)
 {
     const S10Defaults *d = &t->defaults;
     int i;

     if((d->cruise_speed_mps <= 0.0) || (d->scan_spacing_m <= 0.0) ||
        (d->detour_margin_m <= 0.0) || (d->cruise_alt_m < 0.0) ||
        (d->pickup_hold_s < 0) || (d->dropoff_hold_s < 0))
     {
         return (false);
     }

     for(i = 0; i < t->area_count; i++)
     {
         const S10Rect *r = &t->areas[i].rect;
         if((r->x_min >= r->x_max) || (r->y_min >= r->y_max)) { return (false); }
     }
     for(i = 0; i < t->no_fly_count; i++)
     {
         const S10Rect *r = &t->no_fly[i].rect;
         if((r->x_min >= r->x_max) || (r->y_min >= r->y_max)) { return (false); }
     }
     for(i = 0; i < t->gps_denied_count; i++)
     {
         const S10Rect *r = &t->gps_denied[i].rect;
         if((r->x_min >= r->x_max) || (r->y_min >= r->y_max)) { return (false); }
     }
     for(i = 0; i < t->route_count; i++)
     {
         if(t->routes[i].point_count < 1) { return (false); }
     }

     return (true);
 }


 static bool wp_add(WpList *l, int node_id, PrimitiveType action, double x,
                    double y, double z, double speed, int hold_s,
                    const char *label)
 {
     Waypoint *w;

     if(l->count >= S10_MAX_WAYPOINTS)
     {
         l->overflow = true;
         return (false);
     }

     w = &l->items[l->count++];
     w->source_node_id = node_id;
     w->action = action;
     w->position.x = x;
     w->position.y = y;
     w->position.z = z;
     w->speed_mps = speed;
     w->hold_s = hold_s;
     snprintf(w->label, sizeof(w->label), "%s", label);

     return (true);
 }


 /* Resolves a name to an x,y: point -> itself, area -> entry corner,
  * route -> its first point. */
 static bool resolve_xy(const ScenarioTable *t, const char *text,
                        double *x, double *y, S10Resolved *res)
 {
     if(!s10_resolve_location(t, text, res))
     {
         return (false);
     }

     switch(res->kind)
     {
         case S10_LOC_POINT:
             *x = t->points[res->index].x;
             *y = t->points[res->index].y;
             return (true);

         case S10_LOC_AREA:
             *x = t->areas[res->index].rect.x_min;
             *y = t->areas[res->index].rect.y_min;
             return (true);

         case S10_LOC_ROUTE:
             *x = t->routes[res->index].pts[0].x;
             *y = t->routes[res->index].pts[0].y;
             return (true);

         default:
             return (false);
     }
 }


 static S10GenResult gen_fail(S10GenStatus status, int node_id,
                              const char *fmt, const char *arg)
 {
     S10GenResult r;

     memset(&r, 0, sizeof(r));
     r.status = status;
     r.node_id = node_id;
     snprintf(r.detail, sizeof(r.detail), fmt, arg);

     return (r);
 }