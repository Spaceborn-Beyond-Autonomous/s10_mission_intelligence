#ifndef COMMAND_INTERPRETER_H
#define COMMAND_INTERPRETER_H
#include "intent_parser.h"

typedef enum { GOTO, HOVER, SCAN, PICKUP, DROPOFF, LAND_CMD, RTH, PATROL_CMD } PrimitiveType;

typedef struct {
    PrimitiveType type;
    char position[64];
    int  duration_s;
    char area[64];
    char payload[32];
    char route_or_area[64];
    int  repetitions;
} Command;

// Applies the hardcoded recipe for intent.type and returns a flat, ordered
// array of Commands via out_count. Caller owns the returned array and must
// free() it. If intent.type == UNKNOWN, returns NULL and sets *out_count = 0.
Command *interpret_intent(Intent intent, int *out_count);

#endif