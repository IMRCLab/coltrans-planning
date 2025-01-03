#pragma once

#include <ompl/base/OptimizationObjective.h>
#include "robots.h"

namespace ob = ompl::base;

// Class to optimize the cable angles to a desired value
class minCableObjective : public ob::OptimizationObjective
{
public:
    minCableObjective(const ob::SpaceInformationPtr &si, const plannerSettings& cfg);

    ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override;
    ob::Cost stateCost(const ob::State*) const override;

protected:
    float requiredForceMagnitude(const ob::State* s) const;

    const plannerSettings cfg_;
};


class minAngleObjective : public ob::OptimizationObjective
{
public:
    minAngleObjective(const ob::SpaceInformationPtr &si, const unicycleSettings& cfg);

    ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override;
    ob::Cost stateCost(const ob::State*) const override;

protected:

    const unicycleSettings cfg_;
};
