#include "goal.h"
#include "robots.h"

namespace ob = ompl::base;

RobotsWithPayloadGoal::RobotsWithPayloadGoal(const ob::SpaceInformationPtr &si, ompl::base::State* goalState)
    : ob::GoalSampleableRegion(si)
{
    goalState_ = si_->cloneState(goalState);
    auto ss_typed = si_->getStateSpace()->as<StateSpace>();
    // set the values of cable part to be zero
    auto goalState_typed = goalState_->as<StateSpace::StateType>();
    auto cables_typed = goalState_typed->as<ob::RealVectorStateSpace::StateType>(2);
    for (size_t i = 0; i < ss_typed->getNumCables(); ++i) {
        cables_typed->values[i] = 0.0;
    }

    tmpState_ = si_->allocState();

    sampler_ = si_->allocStateSampler();
}

RobotsWithPayloadGoal::~RobotsWithPayloadGoal()
{
    si_->freeState(goalState_);
    si_->freeState(tmpState_);
}

// bool RobotsWithPayloadGoal::isSatisfied(const State *st) const
// {
//     auto st_typed = st->as<StateSpace::StateType>();
// }

double RobotsWithPayloadGoal::distanceGoal(const ompl::base::State *st) const
{
    si_->copyState(tmpState_, st);
    // set the values of cable part to be zero
    auto tmpState_typed = tmpState_->as<StateSpace::StateType>();
    auto cables_typed = tmpState_typed->as<ob::RealVectorStateSpace::StateType>(2);
    auto ss_typed = si_->getStateSpace()->as<StateSpace>();
    for (size_t i = 0; i < ss_typed->getNumCables(); ++i) {
        cables_typed->values[i] = 0.0;
    }

    // si_->printState(tmpState_);
    // si_->printState(goalState_);
    // std::cout << si_->distance(tmpState_, goalState_) << std::endl;

    return si_->distance(tmpState_, goalState_);
}

void RobotsWithPayloadGoal::sampleGoal(ompl::base::State *st) const
{
    sampler_->sampleUniform(st);
    // set the values of the payload pose to match our actual goal
    auto st_typed = st->as<StateSpace::StateType>();
    auto goalState_typed = goalState_->as<StateSpace::StateType>();

    st_typed->setPayloadPos(goalState_typed->getPayloadPos());
    st_typed->setPayloadquat(goalState_typed->getPayloadquat());
}
 
unsigned int RobotsWithPayloadGoal::maxSampleCount() const
{
    return std::numeric_limits<unsigned int>::max();
}