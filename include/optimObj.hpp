// Spaces
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Objectives
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/OptimizationObjective.h>

#include <robots.hpp>
#include <helper.hpp>
// Objective to minimize the path length
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

// Class to optimize the cable angles to a desired value
class minCableObjective : public ob::OptimizationObjective
{
public:
    minCableObjective(const ob::SpaceInformationPtr &si) :
        ob::OptimizationObjective(si)
        {}

    ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override
    {   
        float E1 = stateCost(s1).value();
        float E2 = stateCost(s2).value();
        float c = (E1+E2)/2;

        auto st1 = s1->as<StateSpace::StateType>();
        auto st2 = s2->as<StateSpace::StateType>();
        Eigen::Vector3f pos1 = st1->getPayloadPos();
        Eigen::Vector3f pos2 = st2->getPayloadPos();

        float distance = (pos1 - pos2).norm();

        return ob::Cost(c*distance);
    }

    ob::Cost stateCost(const ob::State* s) const override
    {
        auto st = s->as<StateSpace::StateType>();
        size_t num_cables = si_->getStateSpace()->as<StateSpace>()->getNumCables();
        Eigen::Vector3f basevec(0,0,1);
        float cost = 0;
        for(size_t i = 0; i < num_cables; ++i) {
            auto currentUnitVec =  st->getunitvec(i);
            auto angle = acos(basevec.dot(currentUnitVec));
            cost += abs(angle);
        }
        return ob::Cost(cost);
    }
};
