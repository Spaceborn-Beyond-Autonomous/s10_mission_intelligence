#ifndef S10_MISSION_BUILDER_H
#define S10_MISSION_BUILDER_H

#include "command_interpreter.h"
#include "../../include/s10_graph.h"

// ---------- build ----------
MissionGraph build_mission(Command *commands, int count);

// ---------- adapt to changed conditions ----------
// Event created by Member 4's adapters, passed in by Member 1.
typedef enum {
    S10_CHANGE_WEATHER, S10_CHANGE_COMMS, S10_CHANGE_BATTERY,
    S10_CHANGE_AREA_BLOCKED, S10_CHANGE_OTHER
} S10ChangeType;

typedef enum { S10_SEV_LOW, S10_SEV_MEDIUM, S10_SEV_HIGH, S10_SEV_CRITICAL } S10Severity;

typedef struct {
    S10ChangeType type;
    S10Severity severity;
    int affected_node_id;        // -1 if not tied to a node
    char affected_area[64];      // "" if not tied to an area
} S10ChangeEvent;

typedef enum {
    S10_APPLY_CHANGED,           // out holds an adapted graph
    S10_APPLY_UNCHANGED,         // out holds an identical copy
    S10_APPLY_BAD_ARGS,          // NULL pointer or current_node_id out of range
    S10_APPLY_BAD_INPUT_GRAPH,   // old graph fails s10_graph_validate
    S10_APPLY_NO_MEMORY,
    S10_APPLY_INVALID_RESULT     // adapted graph failed validation (out is empty)
} S10ApplyStatus;

// Adapts the mission graph to a changed condition. `old` is never modified, so the caller
// can keep it (PX4-style: the original plan survives until a new one is accepted).
// Nodes before current_node_id are history and are kept.
// LOW = no change; MEDIUM = 30 s HOVER inserted before the current node;
// HIGH = cut at the first pending node matching the event, then RTH;
// CRITICAL = drop all pending nodes, then LAND_CMD (battery) or RTH.
// On CHANGED/UNCHANGED the caller owns *out and frees it with s10_graph_free.
S10ApplyStatus s10_graph_apply_change(const MissionGraph *old, int current_node_id,
                                      const S10ChangeEvent *ev, MissionGraph *out);

#endif