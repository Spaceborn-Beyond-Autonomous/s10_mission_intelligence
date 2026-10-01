#include "s10_fsm.h"

int s10_fsm_can_transition(
    S10FsmState current,
    S10FsmState next)
{
    if (current == S10_FSM_IDLE &&
        next == S10_FSM_PLANNING)
    {
        return 1;
    }

    if (current == S10_FSM_PLANNING &&
        (next == S10_FSM_EXECUTING ||
         next == S10_FSM_FAILED))
    {
        return 1;
    }

    if (current == S10_FSM_EXECUTING &&
        (next == S10_FSM_REPLANNING ||
         next == S10_FSM_COMPLETED ||
         next == S10_FSM_FAILED))
    {
        return 1;
    }

    if (current == S10_FSM_REPLANNING &&
        (next == S10_FSM_EXECUTING ||
         next == S10_FSM_FAILED))
    {
        return 1;
    }

    return 0;
}