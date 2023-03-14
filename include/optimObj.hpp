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
// Objective to minimize the path length
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

// Class to optimize the cable angles to a desired value
class minCableObjective : public ob::OptimizationObjective
{
public:
    minCableObjective(const ob::SpaceInformationPtr &si, const std::vector<double> &desiredCableStates) :
        ob::OptimizationObjective(si),
        desiredCableStates_(desiredCableStates)
        {}

    ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override
    {
        return combineCosts(stateCost(s1), stateCost(s2));
    }

    ob::Cost stateCost(const ob::State* s) const override
    {
        auto st = s->as<StateSpace::StateType>();
        size_t num_cables = si_->getStateSpace()->as<StateSpace>()->getNumCables();
        Eigen::Vector3f basevec(0,1,0);
        float cost = 0;
        for(size_t i = 0; i < num_cables; ++i) {
            auto currentUnitVec =  st->getunitvec(i);
            auto angle = acos(basevec.dot(currentUnitVec));
            cost += abs(angle);
            // std::cout <<"Cost: "<< cost <<std::endl;
        }
        return ob::Cost(cost);
    }

private: 
    std::vector<double> desiredCableStates_;
};
