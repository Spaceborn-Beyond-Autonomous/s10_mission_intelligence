#ifndef S10_INTERFACE_H
#define S10_INTERFACE_H

#include <stdint.h>

typedef enum
{
    S10_OUTPUT_MISSION_GRAPH = 0,
    S10_OUTPUT_WAYPOINTS,
    S10_OUTPUT_EVENT
} S10OutputType;

typedef struct
{
    uint32_t output_id;
    S10OutputType type;
} S10Output;

#endif
