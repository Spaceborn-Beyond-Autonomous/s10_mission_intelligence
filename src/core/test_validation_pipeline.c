#include "s10_constraints.h"
#include "s10_risk.h"
#include "s10_fsm.h"
#include <stdio.h>
int s10_decision_is_valid(const S10Constraint *constraint);
int s10_safety_check(const S10Constraint *constraint);
int main(void)
{
    S10Constraint ok = {1, S10_CONSTRAINT_OK};
    S10Constraint bad = {2, S10_CONSTRAINT_VIOLATED};
    int failures = 0;

    if (!s10_constraint_is_valid(&ok) ||
        !s10_decision_is_valid(&ok) ||
        !s10_safety_check(&ok))
        failures++;

    if (s10_constraint_is_valid(&bad) ||
        s10_decision_is_valid(&bad) ||
        s10_safety_check(&bad))
        failures++;

    if (s10_risk_level_from_score(0.1f) != S10_RISK_LOW)
        failures++;

    if (!s10_fsm_can_transition(S10_FSM_IDLE, S10_FSM_PLANNING) ||
        !s10_fsm_can_transition(S10_FSM_EXECUTING, S10_FSM_REPLANNING) ||
        s10_fsm_can_transition(S10_FSM_IDLE, S10_FSM_COMPLETED))
        failures++;

    printf("Validation pipeline checks: %s (%d failures)\n",
           failures == 0 ? "PASS" : "FAIL", failures);
    return failures == 0 ? 0 : 1;
}