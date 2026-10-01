#include "s10_constraints.h"

int s10_decision_is_valid(
    const S10Constraint *constraint)
{
    if (constraint == 0)
    {
        return 0;
    }

    return s10_constraint_is_valid(constraint);
}