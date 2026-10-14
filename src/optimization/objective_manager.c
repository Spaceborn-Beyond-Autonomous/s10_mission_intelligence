/**
 * @file    objective_manager.c
 * @author  Aya Ashraf
 * @brief   Implements loading and validation of objectives.yaml.
 *
 * @details
 * Every section of objectives.yaml has the shape  section -> key -> number,
 * so the libyaml event loop needs only a stack of mapping frames.
 */

 #include "objective_manager.h"

 #include <ctype.h>
 #include <math.h>
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <yaml.h>
 
 /************************************* Private Macros Part ************************************* */
 
 #define YAML_MAX_DEPTH   4
 #define YAML_KEY_LEN     32
 
 /************************************* Private Data Types Part ************************************* */
 
 typedef struct
 {
     bool expecting_key;
     char key[YAML_KEY_LEN];
 
 } YamlFrame;
 
 /************************************* Private Function Prototypes ************************************* */
 
 static bool parse_number(const char *text, double *value);
 static bool apply_value(ObjectiveConfig *c, const YamlFrame *f, double v);
 
 /************************************* Public Function Definitions ************************************* */
 
 bool s10_objectives_init(ObjectiveConfig *cfg)
 {
     if(cfg == NULL)
     {
         return (false);
     }
 
     memset(cfg, 0, sizeof(*cfg));
 
     return (true);
 }
 
 
 bool s10_objectives_validate(const ObjectiveConfig *cfg)
 {
     int i;
 
     if(cfg == NULL)
     {
         return (false);
     }
 
     if((cfg->w_fuel < 0.0) || (cfg->w_time < 0.0) || (cfg->w_risk < 0.0) ||
        ((cfg->w_fuel + cfg->w_time + cfg->w_risk) <= 0.0))
     {
         return (false);
     }
 
     if((cfg->fuel_ref <= 0.0) || (cfg->time_ref_s <= 0.0) ||
        (cfg->fuel_ref_speed_mps <= 0.0))
     {
         return (false);
     }
 
     if((cfg->fuel_per_m < 0.0) || (cfg->fuel_speed_exponent < 0.0) ||
        (cfg->fuel_per_hover_s < 0.0) || (cfg->gps_denied_risk_per_s < 0.0))
     {
         return (false);
     }
 
     if((cfg->profile_count < 1) || (cfg->profile_count > S10_MAX_SPEED_PROFILES))
     {
         return (false);
     }
 
     for(i = 0; i < cfg->profile_count; i++)
     {
         if((cfg->profiles[i].factor <= 0.0) ||
            (cfg->profiles[i].name[0] == '\0'))
         {
             return (false);
         }
     }
 
     return (true);
 }
 
 
 bool s10_objectives_load(ObjectiveConfig *cfg, const char *path)
 {
     FILE            *file;
     yaml_parser_t    parser;
     yaml_event_t     event;
     YamlFrame        frames[YAML_MAX_DEPTH];
     int              depth = 0;
     bool             parsing = true;
     bool             ok = false;
     ObjectiveConfig  tmp;
 
     if(cfg == NULL)
     {
         return (false);
     }
 
     (void)s10_objectives_init(cfg);
 
     if(path == NULL)
     {
         return (false);
     }
 
     (void)s10_objectives_init(&tmp);
 
     file = fopen(path, "r");
     if(file == NULL)
     {
         return (false);
     }
 
     if(!yaml_parser_initialize(&parser))
     {
         fclose(file);
         return (false);
     }
 
     yaml_parser_set_input_file(&parser, file);
 
     while(parsing)
     {
         if(!yaml_parser_parse(&parser, &event))
         {
             fprintf(stderr, "s10_objectives_load: %s: line %zu: %s\n", path,
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
             {
                 if(depth >= YAML_MAX_DEPTH)
                 {
                     yaml_event_delete(&event);
                     goto cleanup;
                 }
 
                 frames[depth].expecting_key = true;
                 frames[depth].key[0] = '\0';
                 depth++;
                 break;
             }
 
             case YAML_MAPPING_END_EVENT:
             {
                 if(depth == 0)
                 {
                     yaml_event_delete(&event);
                     goto cleanup;
                 }
 
                 depth--;
 
                 if(depth > 0)
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
 
                 if(frames[depth - 1].expecting_key)
                 {
                     if(event.data.scalar.length >= YAML_KEY_LEN)
                     {
                         yaml_event_delete(&event);
                         goto cleanup;
                     }
 
                     memcpy(frames[depth - 1].key, value,
                            event.data.scalar.length + 1U);
                     frames[depth - 1].expecting_key = false;
 
                     /* A key under speed_profiles names a new profile. */
                     if((depth == 2) &&
                        (strcmp(frames[0].key, "speed_profiles") == 0))
                     {
                         int i;
 
                         if((event.data.scalar.length >= S10_PROFILE_NAME_LEN) ||
                            (tmp.profile_count >= S10_MAX_SPEED_PROFILES))
                         {
                             fprintf(stderr, "s10_objectives_load: %s: bad or "
                                     "too many speed profiles\n", path);
                             yaml_event_delete(&event);
                             goto cleanup;
                         }
 
                         for(i = 0; i < tmp.profile_count; i++)
                         {
                             if(strcmp(tmp.profiles[i].name, value) == 0)
                             {
                                 fprintf(stderr, "s10_objectives_load: %s: "
                                         "duplicate profile '%s'\n", path,
                                         value);
                                 yaml_event_delete(&event);
                                 goto cleanup;
                             }
                         }
 
                         memcpy(tmp.profiles[tmp.profile_count].name, value,
                                event.data.scalar.length + 1U);
                         tmp.profile_count++;
                     }
                 }
                 else
                 {
                     double v;
 
                     /* Values are only allowed one level below a section. */
                     if((depth != 2) || !parse_number(value, &v) ||
                        !apply_value(&tmp, frames, v))
                     {
                         fprintf(stderr, "s10_objectives_load: %s: bad value "
                                 "'%s' for '%s'\n", path, value,
                                 frames[depth - 1].key);
                         yaml_event_delete(&event);
                         goto cleanup;
                     }
 
                     frames[depth - 1].expecting_key = true;
                 }
                 break;
             }
 
             default:
             {
                 /* Sequences, aliases and anything else are not part of the format. */
                 if((event.type == YAML_SEQUENCE_START_EVENT) ||
                    (event.type == YAML_ALIAS_EVENT))
                 {
                     yaml_event_delete(&event);
                     goto cleanup;
                 }
                 break;
             }
         }
 
         yaml_event_delete(&event);
     }
 
     if(!s10_objectives_validate(&tmp))
     {
         fprintf(stderr, "s10_objectives_load: %s: missing or invalid "
                 "values\n", path);
         goto cleanup;
     }
 
     memcpy(cfg, &tmp, sizeof(*cfg));
     ok = true;
 
 cleanup:
 
     yaml_parser_delete(&parser);
     fclose(file);
 
     return (ok);
 }
 
 /************************************* Private Function Definitions ************************************* */
 
 static bool parse_number(const char *text, double *value)
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
 
 
 /* Stores one value; the frames say which section and key it belongs to. */
 static bool apply_value(ObjectiveConfig *c, const YamlFrame *f, double v)
 {
     const char *section = f[0].key;
     const char *key = f[1].key;
 
     if(strcmp(section, "weights") == 0)
     {
         if(strcmp(key, "fuel") == 0) { c->w_fuel = v; return (true); }
         if(strcmp(key, "time") == 0) { c->w_time = v; return (true); }
         if(strcmp(key, "risk") == 0) { c->w_risk = v; return (true); }
         return (false);
     }
 
     if(strcmp(section, "references") == 0)
     {
         if(strcmp(key, "fuel") == 0)   { c->fuel_ref = v;   return (true); }
         if(strcmp(key, "time_s") == 0) { c->time_ref_s = v; return (true); }
         return (false);
     }
 
     if(strcmp(section, "cost") == 0)
     {
         if(strcmp(key, "fuel_per_m") == 0)            { c->fuel_per_m = v;            return (true); }
         if(strcmp(key, "fuel_ref_speed_mps") == 0)    { c->fuel_ref_speed_mps = v;    return (true); }
         if(strcmp(key, "fuel_speed_exponent") == 0)   { c->fuel_speed_exponent = v;   return (true); }
         if(strcmp(key, "fuel_per_hover_s") == 0)      { c->fuel_per_hover_s = v;      return (true); }
         if(strcmp(key, "gps_denied_risk_per_s") == 0) { c->gps_denied_risk_per_s = v; return (true); }
         return (false);
     }
 
     if(strcmp(section, "speed_profiles") == 0)
     {
         c->profiles[c->profile_count - 1].factor = v;
         return (true);
     }
 
     return (false);
 }