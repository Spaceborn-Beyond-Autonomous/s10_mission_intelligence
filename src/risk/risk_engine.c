#include "s10_risk.h"

S10RiskLevel s10_risk_level_from_score(
    float score)
{
    if (score < 0.25f)
    {
        return S10_RISK_LOW;
    }

    if (score < 0.50f)
    {
        return S10_RISK_MEDIUM;
    }

    if (score < 0.75f)
    {
        return S10_RISK_HIGH;
    }

    return S10_RISK_CRITICAL;
}
