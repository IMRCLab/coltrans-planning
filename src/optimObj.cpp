#include "optimObj.h"
#include "robots.h"

minCableObjective::minCableObjective(const ob::SpaceInformationPtr &si) :
    ob::OptimizationObjective(si)
    {}

float minCableObjective::requiredForceMagnitude(const ob::State* s) const
{
    auto st = s->as<StateSpace::StateType>();
    size_t num_cables = si_->getStateSpace()->as<StateSpace>()->getNumCables();
    Eigen::Vector3f basevec(0,0,1);
    float cost = 0;        
    for(size_t i = 0; i < num_cables; ++i) {
        auto currentUnitVec =  st->getunitvec(i);
        cost += (1.0f/(basevec.dot(currentUnitVec)));
    }
    cost *= (1.0/num_cables);
    return cost;
}

ob::Cost minCableObjective::motionCost(const ob::State *s1, const ob::State *s2) const
{   
    float E1 = requiredForceMagnitude(s1);
    float E2 = requiredForceMagnitude(s2);

    auto st1 = s1->as<StateSpace::StateType>();
    auto st2 = s2->as<StateSpace::StateType>();
    Eigen::Vector3f pos1 = st1->getPayloadPos();
    Eigen::Vector3f pos2 = st2->getPayloadPos();
    float distance = (pos1 - pos2).norm();

    float c = (E1+E2)/2 * distance;
    return ob::Cost(c);
}

ob::Cost minCableObjective::stateCost(const ob::State*) const
{
    return identityCost();
}


