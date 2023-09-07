#include <ompl/util/Exception.h>
#include <ompl/base/SpaceInformation.h>

#include "sampler.h"
#include "fclStateValidityChecker.h"

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
    sampler_cables_ = ss_typed->getSubspace(2)->allocStateSampler();


    // sample a number of valid cable states
    std::cout << "Generating valid cable states..." << std::endl;

    fclStateValidityChecker checker(si, robots, std::make_shared<Obstacles>(), cfg);


    for (size_t i = 0; i < 100; ++i) {
        auto state = ss->allocState();
        // sampleValidCables(state);
        std::cout << i << std::endl;
        do {
            ob::State **comps = state->as<ob::CompoundState>()->components;
            sampler_pos_->sampleUniform(comps[0]);
            sampler_orientation_->sampleUniform(comps[1]);
            sampler_cables_->sampleUniform(comps[2]);
        } while (!checker.isValid(state));

        valid_cable_states_.push_back(state);
    }
    std::cout << "Done" << std::endl;

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
    
    
    #if 0
    sampler_pos_->sampleUniform(comps[0]);
    sampler_orientation_->sampleUniform(comps[1]);
    // sampler_cables_->sampleUniform(comps[2]);
    sampleValidCables(state);
    #endif

    int idx = rng_.uniformInt(0, valid_cable_states_.size() - 1);
    space_->copyState(state, valid_cable_states_[idx]);
    sampler_pos_->sampleUniform(comps[0]);
    sampler_orientation_->sampleUniform(comps[1]);


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
