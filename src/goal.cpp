#include "goal.h"
#include "robots.h"

namespace ob = ompl::base;

RobotsWithPayloadGoal::RobotsWithPayloadGoal(const ob::SpaceInformationPtr &si, const ompl::base::SE3StateSpace::StateType& goal)
    : ob::GoalSampleableRegion(si)
    , goal_(goal)
{
}

bool RobotsWithPayloadGoal::isSatisfied(const State *st) const
{

}

double RobotsWithPayloadGoal::distanceGoal(const State *st) const
{

}

void RobotsWithPayloadGoal::sampleGoal(State *st) const
{

}
 
unsigned int RobotsWithPayloadGoal::maxSampleCount() const
{

}