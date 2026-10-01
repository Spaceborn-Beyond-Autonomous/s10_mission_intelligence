#include "s10_constraints.h"

int s10_safety_check(
    const S10Constraint *constraint)
{
    if (constraint == 0)
    {
        return 0;
    }

    if (constraint->status == S10_CONSTRAINT_OK)
    {
        return 1;
    }

    return 0;
}
