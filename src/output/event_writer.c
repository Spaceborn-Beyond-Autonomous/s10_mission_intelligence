#include "s10_fsm.h"

int s10_event_write(
    const S10FsmEvent *event)
{
    if (event == 0)
    {
        return 0;
    }

    return 1;
}
