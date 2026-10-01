#ifndef INTENT_PARSER_H
#define INTENT_PARSER_H

#include <stdint.h>

// UNKNOWN=0 is deliberate: an unrecognized command must not silently
// look like a valid SURVEY (which Intent intent = {0}; would otherwise
// produce, since SURVEY would default to the enum's zero value)
typedef enum { UNKNOWN = 0, SURVEY, DELIVER, PATROL, RETURN_TO_BASE, LAND } IntentType;

typedef struct {
    IntentType type;            // which intent this is; determines which fields below are meaningful

    char area[64];               // SURVEY only: region to scan
    char pickup_location[64];    // DELIVER only: where to grab the payload
    char dropoff_location[64];   // DELIVER only: where to drop the payload
    char payload[32];            // DELIVER only: what's being carried (optional, "carrying <payload>")

    char route_or_area[64];      // PATROL only: named route or region to loop over
    int  repetitions;            // PATROL only: number of loops

    char location[64];           // LAND only, optional: empty = land at current position

    int  priority;               // used across multiple intents
} Intent;

Intent parse_intent(const char *text);

#endif