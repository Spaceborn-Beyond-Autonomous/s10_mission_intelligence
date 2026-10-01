#ifndef S10_RISK_H
#define S10_RISK_H

#include <stdint.h>

typedef enum
{
    S10_RISK_LOW = 0,
    S10_RISK_MEDIUM,
    S10_RISK_HIGH,
    S10_RISK_CRITICAL
} S10RiskLevel;

typedef struct
{
    uint32_t risk_id;
    S10RiskLevel level;
    float score;
} S10Risk;
S10RiskLevel s10_risk_level_from_score(
    float score);
float s10_risk_estimate(
    float base_risk,
    float constraint_risk);
#endif
