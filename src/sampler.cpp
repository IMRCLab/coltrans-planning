#include <ompl/util/Exception.h>
#include <ompl/base/SpaceInformation.h>

#include "sampler.h"
#include "fclStateValidityChecker.h"
#include "helper.h"

#include "assignment.hpp"

namespace ob = ompl::base;

// static, so we don't have to recompute multiple times
std::vector<ompl::base::State *> RobotsWithPayloadStateSampler::valid_cable_states_;

RobotsWithPayloadStateSampler::RobotsWithPayloadStateSampler(
    ompl::base::SpaceInformationPtr si,
    const ob::StateSpace *ss,
    std::shared_ptr<RobotsWithPayload> robots,
    const plannerSettings& cfg)
    : StateSampler(ss)
    , robots_(robots)
    , cfg_(cfg)
    , uniform_(false)
{
    auto ss_typed = ss->as<StateSpace>();

    sampler_pos_ = ss_typed->getSubspace(0)->allocStateSampler();
    sampler_orientation_ = ss_typed->getSubspace(1)->allocStateSampler();
    if (ss_typed->getNumCables() > 0) {
        sampler_cables_ = ss_typed->getSubspace(2)->allocStateSampler();
    }

    if (cfg_.sampler == "uniform") {
        uniform_ = true;
    } else if (valid_cable_states_.size() == 0) {

        // sample a number of valid cable states
        const size_t num_cable_states = 1000;
        std::cout << "Generating " << num_cable_states << " cable states..." << std::endl;

        // fclStateValidityChecker checker(si, robots, std::make_shared<Obstacles>(), cfg);

        auto startState = si->allocState();
        si->getStateSpace()->copyFromReals(startState, eigentoStd(cfg.start));
        valid_cable_states_.push_back(startState);


        for (size_t i = 0; i < num_cable_states; ++i) {
            // take the position/orientation from the start to guarantee collision-free operation
            auto state = ss->cloneState(startState);
            // sampleValidCables(state);
            // std::cout << i << std::endl;
            bool motionValid;
            do {
                ob::State **comps = state->as<ob::CompoundState>()->components;
                // sampler_pos_->sampleUniform(comps[0]);
                // sampler_orientation_->sampleUniform(comps[1]);
                if (ss_typed->getNumCables() > 0) {
                    sampler_cables_->sampleUniform(comps[2]);
                }
                // if (ss_typed->getNumCables() > 0) {
                //     int idx = rng_.uniformInt(0, valid_cable_states_.size() - 1);
                //     sampler_cables_->sampleGaussian(comps[2], valid_cable_states_[idx]->as<ob::CompoundState>()->components[2], 0.5);
                // }

                // re-arrange to avoid factorial permutation
                // a) pick a random existing state
                int idx = rng_.uniformInt(0, valid_cable_states_.size() - 1);
                auto baseState = valid_cable_states_[idx];
                // b) compute cost between UAVs

                auto st1 = baseState->as<StateSpace::StateType>();
                auto st2 = state->as<StateSpace::StateType>();

                libMultiRobotPlanning::Assignment<size_t, size_t> assignment;
                for (size_t i = 0; i < ss_typed->getNumCables(); ++i) {

                    double length1 = cfg_.cablelengthVec[i];
                    Eigen::Vector3f attachmentPoint1(cfg_.attachmentpoints(0+3*i), cfg_.attachmentpoints(1+3*i), cfg_.attachmentpoints(2+3*i));
                    Eigen::Vector3f pos1 = st1->getuavPos(i, attachmentPoint1, length1);

                    for (size_t j = 0; j < ss_typed->getNumCables(); ++j) {

                        double length2 = cfg_.cablelengthVec[j];
                        Eigen::Vector3f attachmentPoint2(cfg_.attachmentpoints(0+3*j), cfg_.attachmentpoints(1+3*j), cfg_.attachmentpoints(2+3*j));
                        Eigen::Vector3f pos2 = st2->getuavPos(j, attachmentPoint2, length2);

                        double dist = (pos1 - pos2).norm();
                        int dist_in_mm = dist * 1000;
                        assignment.setCost(i, j, dist_in_mm);
                        // std::cout << i << " " << j << " " << dist << " " << pos1 << " " << pos2 << std::endl;
                    }
                }

                std::map<size_t, size_t> solution;
                int64_t c = assignment.solve(solution);

                // std::cout << "solution with cost: " << c << std::endl;
                // for (const auto& s : solution) {
                //     std::cout << s.first << ": " << s.second << std::endl;
                // }

                // re-arrange
                auto cablestate = state->as<ob::CompoundState>()->components[2]->as<ob::RealVectorStateSpace::StateType>();
                std::vector<float> vec;
                for (int i = 0; i < 2*ss_typed->getNumCables(); ++i) {
                    vec.push_back(cablestate->values[i]);
                }

                for (const auto& s : solution) {
                    cablestate->values[2*s.first+0] = vec[2*s.second+0];
                    cablestate->values[2*s.first+1] = vec[2*s.second+1]; 
                }

                motionValid = si->checkMotion(baseState, state);
                // std::cout << m << std::endl;

            } while (!si->isValid(state) || !motionValid);

            valid_cable_states_.push_back(state);
            // si->printState(state);
        }
        std::cout << "Done" << std::endl;
        uniform_ = false;
    }

    std::cout << "uniform sampling: " << uniform_ << std::endl;
}

#if 0
void RobotsWithPayloadStateSampler::sampleValidCables(ompl::base::State *state)
{
    ob::State **comps = state->as<ob::CompoundState>()->components;

    auto space_typed = space_->as<StateSpace>();
    auto& bounds = space_typed->getCableBounds();

    auto cablestate = comps[2]->as<ob::RealVectorStateSpace::StateType>();

    robots_->setPayloadTransformation(state, cfg_.payloadShape);
    robots_->col_mgr_all->clear();
    robots_->col_mgr_all->registerObject(robots_->payloadObj);
    robots_->col_mgr_cables->clear();

    for (int i = 0; i < space_typed->getNumCables(); ) {
        cablestate->values[0+2*i] = rng_.uniformReal(bounds.low[0+2*i], bounds.high[0+2*i]);
        cablestate->values[1+2*i] = rng_.uniformReal(bounds.low[1+2*i], bounds.high[1+2*i]);

        // checking
        double length = cfg_.cablelengthVec[i];
        Eigen::Vector3f attachmentpoint(cfg_.attachmentpoints(0+3*i), cfg_.attachmentpoints(1+3*i), cfg_.attachmentpoints(2+3*i));
        robots_->setUAVTransformation(state, i, attachmentpoint, length);
        robots_->setCableTransformation(state, i, attachmentpoint, length);
        robots_->col_mgr_all->registerObject(robots_->uavObj[i]);
        robots_->col_mgr_all->update();

        // Inter-robot collision 
        fcl::DefaultCollisionData<float> collision_data_robots;
        robots_->col_mgr_all->collide(&collision_data_robots, fcl::DefaultCollisionFunction<float>);
        if (collision_data_robots.result.isCollision()) {
            robots_->col_mgr_all->unregisterObject(robots_->uavObj[i]);
            // std::cout << "try again " << i << std::endl;
            continue;
        }

        robots_->col_mgr_cables->registerObject(robots_->cablesObj[i]);
        robots_->col_mgr_cables->update();
        
        // inter-cable collision
        fcl::DefaultCollisionData<float> collision_data_cables;
        robots_->col_mgr_cables->collide(&collision_data_cables, fcl::DefaultCollisionFunction<float>);
        if (collision_data_cables.result.isCollision()) {
            robots_->col_mgr_all->unregisterObject(robots_->uavObj[i]);
            robots_->col_mgr_cables->unregisterObject(robots_->cablesObj[i]);

            // std::cout << "try again2 " << i << std::endl;


            continue;
        }

        ++i;
    }
}
#endif

void RobotsWithPayloadStateSampler::sampleUniform(ompl::base::State *state)
{
    ob::State **comps = state->as<ob::CompoundState>()->components;
    
    
    if (uniform_) {
        sampler_pos_->sampleUniform(comps[0]);
        sampler_orientation_->sampleUniform(comps[1]);
        sampler_cables_->sampleUniform(comps[2]);
        // sampleValidCables(state);
        return;
    } else {

        sampler_pos_->sampleUniform(comps[0]);
        sampler_orientation_->sampleUniform(comps[1]);

        auto ss_typed = space_->as<StateSpace>();
        if (ss_typed->getNumCables() > 0) {
            int idx = rng_.uniformInt(0, valid_cable_states_.size() - 1);
            sampler_cables_->sampleGaussian(comps[2], valid_cable_states_[idx]->as<ob::CompoundState>()->components[2], 0.05);
        }
    }

    #if 0
    int idx = rng_.uniformInt(0, valid_cable_states_.size() - 1);
    space_->copyState(state, valid_cable_states_[idx]);
    sampler_pos_->sampleUniform(comps[0]);
    sampler_orientation_->sampleUniform(comps[1]);
    #endif

#if 0
    auto space_typed = space_->as<StateSpace>();
    auto& bounds = space_typed->getCableBounds();

    auto cablestate = comps[2]->as<ob::RealVectorStateSpace::StateType>();

    robots_->setPayloadTransformation(state, cfg_.payloadShape);
    robots_->col_mgr_all->clear();
    robots_->col_mgr_all->registerObject(robots_->payloadObj);
    robots_->col_mgr_cables->clear();

    for (int i = 0; i < space_typed->getNumCables(); ) {
        cablestate->values[0+2*i] = rng_.uniformReal(bounds.low[0+2*i], bounds.high[0+2*i]);
        cablestate->values[1+2*i] = rng_.uniformReal(bounds.low[1+2*i], bounds.high[1+2*i]);

        // checking
        double length = cfg_.cablelengthVec[i];
        Eigen::Vector3f attachmentpoint(cfg_.attachmentpoints(0+3*i), cfg_.attachmentpoints(1+3*i), cfg_.attachmentpoints(2+3*i));
        robots_->setUAVTransformation(state, i, attachmentpoint, length);
        robots_->setCableTransformation(state, i, attachmentpoint, length);
        robots_->col_mgr_all->registerObject(robots_->uavObj[i]);
        robots_->col_mgr_all->update();

        // Inter-robot collision 
        fcl::DefaultCollisionData<float> collision_data_robots;
        robots_->col_mgr_all->collide(&collision_data_robots, fcl::DefaultCollisionFunction<float>);
        if (collision_data_robots.result.isCollision()) {
            robots_->col_mgr_all->unregisterObject(robots_->uavObj[i]);
            std::cout << "try again " << i << std::endl;
            continue;
        }

        robots_->col_mgr_cables->registerObject(robots_->cablesObj[i]);
        robots_->col_mgr_cables->update();
        
        // inter-cable collision
        fcl::DefaultCollisionData<float> collision_data_cables;
        robots_->col_mgr_cables->collide(&collision_data_cables, fcl::DefaultCollisionFunction<float>);
        if (collision_data_cables.result.isCollision()) {
            robots_->col_mgr_all->unregisterObject(robots_->uavObj[i]);
            robots_->col_mgr_cables->unregisterObject(robots_->cablesObj[i]);

            std::cout << "try again2 " << i << std::endl;


            continue;
        }

        ++i;
    }
#endif
}

void RobotsWithPayloadStateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance)
{
    throw ompl::Exception("RobotsWithPayloadStateSampler::sampleUniformNear", "not implemented");
}

void RobotsWithPayloadStateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev)
{
    throw ompl::Exception("RobotsWithPayloadStateSampler::sampleGaussian", "not implemented");
}




std::vector<ompl::base::State *> UnicyclesWithRodsStateSampler::valid_formations_;

UnicyclesWithRodsStateSampler::UnicyclesWithRodsStateSampler(
    ompl::base::SpaceInformationPtr si,
    const ob::StateSpace *ss,
    std::shared_ptr<UnicyclesWithRods> robots,
    const unicycleSettings& cfg)
    : StateSampler(ss)
    , robots_(robots)
    , cfg_(cfg)
    , si_(si)
    , uniform_(false)
{
    auto ss_typed = ss->as<UnicyclesStateSpace>();
    sampler_pos_ = ss_typed->getSubspace(0)->allocStateSampler();
    sampler_alpha_ = ss_typed->getSubspace(1)->allocStateSampler();

    if (ss_typed->getNumRobots() > 0) {
        sampler_formations_ = ss_typed->getSubspace(2)->allocStateSampler();
    }
    if (cfg_.sampler == "uniform") {
        uniform_ = true;
    } else if (valid_formations_.size() == 0) {
        const size_t num_formations_states = 1000;
        std::cout << "Generating " << num_formations_states << " formation states..." << std::endl;

        auto startState = si->allocState();
        si->getStateSpace()->copyFromReals(startState, eigentoStd(cfg.start));
        valid_formations_.push_back(startState);

        for (size_t i = 0; i < num_formations_states; ++i) {
            auto state = ss->cloneState(startState);
            bool motionValid;
            do {
                ob::State **comps = state->as<ob::CompoundState>()->components;            
                // ** Sample px, py, and angles (theta1, theta2, ...) **
                if (ss_typed->getNumRobots() > 0) {
                    sampler_formations_->sampleUniform(comps[2]);
                }                
                // ** Select Base State **
                int idx = rng_.uniformInt(0, valid_formations_.size() - 1);
                auto baseState = valid_formations_[idx];

                auto st1 = baseState->as<UnicyclesStateSpace::StateType>();
                auto st2 = state->as<UnicyclesStateSpace::StateType>();

                libMultiRobotPlanning::Assignment<size_t, size_t> assignment;
                for (size_t i = 0; i < ss_typed->getNumRobots(); ++i) {
                    Eigen::Vector3f state1 = st1->getRobotSt(i);
                    Eigen::Vector3f pos1(state1(0), state1(1), 0);

                    for (size_t j = 0; j < ss_typed->getNumRobots(); ++j) {
                        Eigen::Vector3f state2 = st2->getRobotSt(j);
                        Eigen::Vector3f pos2(state2(0), state2(1), 0);
                        double dist = (pos1 - pos2).norm();
                        int dist_in_mm = dist * 1000;
                        assignment.setCost(i, j, dist_in_mm);
                        // std::cout << "i: " << i << ", j: " << j << ", dist: " << dist << ", pos1: (" << pos1(0) << ", " << pos1(1) << ")"  << ", pos2: (" << pos2(0) << ", " << pos2(1) << ")"  << std::endl;
                    }
                }

                std::map<size_t, size_t> solution;
                int64_t c = assignment.solve(solution);

                // std::cout << "solution with cost: " << c << std::endl;
                // for (const auto& s : solution) {
                //     std::cout << "first, second: " << s.first << ": " << s.second << std::endl;
                // }

                // re-arrange
                auto rodstate = state->as<ob::CompoundState>()->components[2]->as<ob::RealVectorStateSpace::StateType>();
                std::vector<float> vec;
                for (int i = 0; i < ss_typed->getNumRobots()-1; ++i) {
                    vec.push_back(rodstate->values[i]);
                }
                for (const auto& s : solution) {
                    // std::cout << "first, second: " << s.first << ": " << s.second << std::endl;
                    if (s.first != 0 && s.second != 0) {
                        rodstate->values[s.first - 1] = vec[s.second - 1];
                    }
                }

                // ** Check Validity **
                motionValid = si->checkMotion(baseState, state);
                
            } while (!si->isValid(state) || !motionValid);

            valid_formations_.push_back(state);
        }
        std::cout << "Done" << std::endl;
        uniform_ = false;
    }
}

void UnicyclesWithRodsStateSampler::sampleUniform(ompl::base::State *state)
{
    ob::State **comps = state->as<ob::CompoundState>()->components;
    auto goalState = si_->allocState();
    si_->getStateSpace()->copyFromReals(goalState, eigentoStd(cfg_.goal));

    if (uniform_) {
        sampler_formations_->sampleUniform(state);
        return;
    } else {

        auto ss_typed = space_->as<UnicyclesStateSpace>();
        sampler_pos_->sampleUniform(comps[0]);
        sampler_alpha_->sampleUniformNear(comps[1], goalState, 0.1);
        if (ss_typed->getNumRobots() > 0) {
            int idx = rng_.uniformInt(0, valid_formations_.size() - 1);
            sampler_formations_->sampleGaussian(comps[2], valid_formations_[idx]->as<ob::CompoundState>()->components[2], 0.05);
        }
    }
}

void UnicyclesWithRodsStateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance)
{
    throw ompl::Exception("UnicyclesWithRodsStateSampler::sampleUniformNear", "not implemented");
}

void UnicyclesWithRodsStateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev)
{
    throw ompl::Exception("UnicyclesWithRodsStateSampler::sampleGaussian", "not implemented");
}