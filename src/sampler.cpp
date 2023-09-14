#include <ompl/util/Exception.h>
#include <ompl/base/SpaceInformation.h>

#include "sampler.h"
#include "fclStateValidityChecker.h"
#include "helper.h"

#include "assignment.hpp"

namespace ob = ompl::base;

RobotsWithPayloadStateSampler::RobotsWithPayloadStateSampler(
    ompl::base::SpaceInformationPtr si,
    const ob::StateSpace *ss,
    std::shared_ptr<RobotsWithPayload> robots,
    const plannerSettings& cfg)
    : StateSampler(ss)
    , robots_(robots)
    , cfg_(cfg)
{
    auto ss_typed = ss->as<StateSpace>();

    sampler_pos_ = ss_typed->getSubspace(0)->allocStateSampler();
    sampler_orientation_ = ss_typed->getSubspace(1)->allocStateSampler();
    if (ss_typed->getNumCables() > 0) {
        sampler_cables_ = ss_typed->getSubspace(2)->allocStateSampler();
    }

    if (cfg_.sampler == "uniform") {
        uniform_ = true;
    } else {
        // sample a number of valid cable states
        std::cout << "Generating valid cable states..." << std::endl;

        fclStateValidityChecker checker(si, robots, std::make_shared<Obstacles>(), cfg);

        auto startState = si->allocState();
        si->getStateSpace()->copyFromReals(startState, eigentoStd(cfg.start));
        valid_cable_states_.push_back(startState);


        for (size_t i = 0; i < 1000; ++i) {
            auto state = ss->allocState();
            // sampleValidCables(state);
            // std::cout << i << std::endl;
            do {
                ob::State **comps = state->as<ob::CompoundState>()->components;
                sampler_pos_->sampleUniform(comps[0]);
                sampler_orientation_->sampleUniform(comps[1]);
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
            } while (!checker.isValid(state));

            valid_cable_states_.push_back(state);
            // si->printState(state);
        }
        std::cout << "Done" << std::endl;
        uniform_ = false;
    }
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
            sampler_cables_->sampleGaussian(comps[2], valid_cable_states_[idx]->as<ob::CompoundState>()->components[2], 0.2);
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
