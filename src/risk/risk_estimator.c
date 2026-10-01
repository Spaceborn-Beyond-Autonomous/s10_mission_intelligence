#include "s10_risk.h"

float s10_risk_estimate(
    float base_risk,
    float constraint_risk)
{
    float score;

    score = (base_risk + constraint_risk) / 2.0f;

    if (score < 0.0f)
    {
        return 0.0f;
    }

    if (score > 1.0f)
    {
        return 1.0f;
    }

    return score;
}