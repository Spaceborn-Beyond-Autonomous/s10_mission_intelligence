#ifndef S10_FSM_H
#define S10_FSM_H

#include <stdint.h>

typedef enum
{
    S10_FSM_IDLE = 0,
    S10_FSM_PLANNING,
    S10_FSM_EXECUTING,
    S10_FSM_REPLANNING,
    S10_FSM_COMPLETED,
    S10_FSM_FAILED
} S10FsmState;

typedef struct
{
    uint32_t event_id;
    S10FsmState state;
} S10FsmEvent;
int s10_fsm_can_transition(
    S10FsmState current,
    S10FsmState next);
#endif
