#ifndef S10_CONSTRAINTS_H
#define S10_CONSTRAINTS_H

#include <stdint.h>

typedef enum
{
    S10_CONSTRAINT_OK = 0,
    S10_CONSTRAINT_VIOLATED,
    S10_CONSTRAINT_INFEASIBLE
} S10ConstraintStatus;

typedef struct
{
    uint32_t constraint_id;
    S10ConstraintStatus status;
} S10Constraint;
int s10_constraint_is_valid(
    const S10Constraint *constraint);
#endif