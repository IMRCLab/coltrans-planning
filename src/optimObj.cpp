#include "optimObj.h"

minCableObjective::minCableObjective(const ob::SpaceInformationPtr &si, const plannerSettings& cfg) :
    ob::OptimizationObjective(si)
    , cfg_(cfg)
    {}

float minCableObjective::requiredForceMagnitude(const ob::State* s) const
{
    auto st = s->as<StateSpace::StateType>();
    size_t num_cables = si_->getStateSpace()->as<StateSpace>()->getNumCables();
    if (num_cables == 0) {
        return 1;
    }
    Eigen::Vector3f basevec(0,0,1);
    float cost = 0;        
    for(size_t i = 0; i < num_cables; ++i) {
        auto currentUnitVec =  st->getunitvec(i);
        cost += (1.0f/(basevec.dot(currentUnitVec.cwiseAbs())));
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

    // compute 
    float distance_uavs = 0;
    Eigen::Vector3f pos1(0,0,0);
    Eigen::Vector3f pos2(0,0,0);
    size_t num_cables = si_->getStateSpace()->as<StateSpace>()->getNumCables();
    if (num_cables > 0) {
        for(size_t i = 0; i<num_cables; ++i) {
            double length = cfg_.cablelengthVec[i];
            Eigen::Vector3f attachmentPoint(cfg_.attachmentpoints(0+3*i), cfg_.attachmentpoints(1+3*i), cfg_.attachmentpoints(2+3*i));
            pos1 += st1->getuavPos(i, attachmentPoint, length);
            pos2 += st2->getuavPos(i, attachmentPoint, length);
        }
        pos1 /= num_cables;
        pos2 /= num_cables;
        distance_uavs = (pos1 - pos2).norm();
    }

    pos1 = st1->getPayloadPos();
    pos2 = st2->getPayloadPos();
    float distance_payload = (pos1 - pos2).norm();

    const float alpha = 0.5;

    float c = (E1+E2)/2 * (alpha * distance_payload + (1-alpha) * distance_uavs);
    return ob::Cost(c);
}

ob::Cost minCableObjective::stateCost(const ob::State*) const
{
    return identityCost();
}


