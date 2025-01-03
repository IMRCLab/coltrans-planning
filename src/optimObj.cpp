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




minAngleObjective::minAngleObjective(const ob::SpaceInformationPtr &si, const unicycleSettings& cfg)
    : ob::OptimizationObjective(si), cfg_(cfg) {}



ob::Cost minAngleObjective::motionCost(const ob::State *s1, const ob::State *s2) const {
   const auto *state1 = s1->as<ob::CompoundState>();
    const auto *state2 = s2->as<ob::CompoundState>();

    // Extract px, py component
    const auto *pos1 = state1->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *pos2 = state2->as<ob::RealVectorStateSpace::StateType>(0);

    // Extract alphas component
    const auto *alphas1 = state1->as<ob::RealVectorStateSpace::StateType>(1);
    const auto *alphas2 = state2->as<ob::RealVectorStateSpace::StateType>(1);

    // Extract thetas component
    const auto *thetas1 = state1->as<ob::RealVectorStateSpace::StateType>(2);
    const auto *thetas2 = state2->as<ob::RealVectorStateSpace::StateType>(2);

    const size_t num_robots = cfg_.num_robots;

    // Path length (distance cost for robot 1)
    double distance_cost = std::sqrt(std::pow(pos2->values[0] - pos1->values[0], 2) +
                                     std::pow(pos2->values[1] - pos1->values[1], 2));

    // Initialize the starting positions
    double px1 = pos1->values[0];
    double py1 = pos1->values[1];
    double px2 = pos2->values[0];
    double py2 = pos2->values[1];
    double l = 0.5; // Assume fixed rod length
    for (size_t i = 0; i < num_robots - 1; ++i) {
        // Compute next robot positions using theta
        px1 += l * cos(thetas1->values[i]);
        py1 += l * sin(thetas1->values[i]);
        px2 += l * cos(thetas2->values[i]);
        py2 += l * sin(thetas2->values[i]);

        // Add the path length for robot i
        distance_cost += std::sqrt(std::pow(px2 - px1, 2) + std::pow(py2 - py1, 2));
    }

    // Theta-Alpha Alignment 
    double alignment_cost = 0.0;
    for (size_t i = 0; i < num_robots - 1; ++i) {
        double theta1 = thetas1->values[i];
        double theta2 = thetas2->values[i];
        double alpha1 = alphas1->values[i];
        double alpha2 = alphas2->values[i];

    // Compute theta change
    double delta_theta = theta2 - theta1;
        double sign_delta_theta = (delta_theta > 0) ? 1.0 : -1.0;

        if (std::abs(delta_theta) > 1e-1) {
            Eigen::Vector2d desired_dir(-sign_delta_theta * std::sin(theta2), 
                                        sign_delta_theta * std::cos(theta2));
            Eigen::Vector2d alpha_dir(std::cos(alpha2), std::sin(alpha2));

            double dot_product = desired_dir.dot(alpha_dir);
            alignment_cost += std::pow(1.0 - dot_product, 2);
        } else {
            alignment_cost += std::pow(alpha2 - theta2, 2);
        }
    }
    // Combine the costs
    double total_cost =  distance_cost + 0.5*alignment_cost;
    return ob::Cost(total_cost);
}

// Cost for a single state (can be used for static penalties or regularization, not used here)
ob::Cost minAngleObjective::stateCost(const ob::State* state) const {
    // No additional state cost; return zero
    return ob::Cost(0.0);
}