#pragma once

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SE3StateSpace.h>

class RobotsWithPayloadGoal : public ompl::base::GoalSampleableRegion
{
public:
    RobotsWithPayloadGoal(const ompl::base::SpaceInformationPtr &si, 
    ompl::base::State* goal);

    ~RobotsWithPayloadGoal() override;
    // bool isSatisfied (const State *st) const override;

    double distanceGoal (const ompl::base::State *st) const override;

    void sampleGoal (ompl::base::State *st) const override;
 
    unsigned int maxSampleCount () const override;

private:
    // ompl::base::SE3StateSpace::StateType goal_;
    ompl::base::State* goalState_;
    ompl::base::State* tmpState_;
    ompl::base::StateSamplerPtr sampler_;
};