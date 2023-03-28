#pragma once

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SE3StateSpace.h>

class RobotsWithPayloadGoal : public ompl::base::GoalSampleableRegion
{
public:
    RobotsWithPayloadGoal(const ompl::base::SpaceInformationPtr &si, 
    const ompl::base::SE3StateSpace::StateType& goal);

    bool isSatisfied (const State *st) const override;

    double distanceGoal (const State *st) const override;

    void sampleGoal (State *st) const override;
 
    unsigned int maxSampleCount () const override;

private:
    ompl::base::SE3StateSpace::StateType goal_;
};