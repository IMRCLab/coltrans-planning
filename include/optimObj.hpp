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
        return this->combineCosts(this->stateCost(s1), this->stateCost(s2));
    }

    ob::Cost stateCost(const ob::State* s) const override
    {
        const auto cablestates = s->as<ob::RealVectorStateSpace::StateType>();      
        float cost = 0;
        for (size_t i = 0; i < 3; ++i) {
        cost += std::pow(desiredCableStates_[i] - cablestates->values[i],2); 
        }
        cost = std::sqrt(cost);
        return ob::Cost(cost);
    }

private: 
    std::vector<double> desiredCableStates_;
};
