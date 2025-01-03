#pragma once

#include <ompl/base/StateSampler.h>
#include <ompl/util/RandomNumbers.h>

#include "robots.h"

class RobotsWithPayloadStateSampler : public ompl::base::StateSampler
{
public:
    RobotsWithPayloadStateSampler(
        ompl::base::SpaceInformationPtr si,
        const ompl::base::StateSpace *ss,
        std::shared_ptr<RobotsWithPayload> robots,
        const plannerSettings& cfg);

    void sampleUniform(ompl::base::State *state) override;

    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;
 
    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;

protected:
    void sampleValidCables(ompl::base::State *state);

protected:
    std::shared_ptr<RobotsWithPayload> robots_;
    const plannerSettings cfg_;

    ompl::base::StateSamplerPtr sampler_pos_;
    ompl::base::StateSamplerPtr sampler_orientation_;
    ompl::base::StateSamplerPtr sampler_cables_;

    static std::vector<ompl::base::State *> valid_cable_states_;

    bool uniform_;

};




class UnicyclesWithRodsStateSampler : public ompl::base::StateSampler
{
public:
    UnicyclesWithRodsStateSampler(
        ompl::base::SpaceInformationPtr si,
        const ompl::base::StateSpace *ss,
        std::shared_ptr<UnicyclesWithRods> robots,
        const unicycleSettings& cfg);

    void sampleUniform(ompl::base::State *state) override;

    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;
 
    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;

protected:
    void sampleValidFormations(ompl::base::State *state);

protected:
    std::shared_ptr<UnicyclesWithRods> robots_;
    const unicycleSettings cfg_;
    ompl::base::SpaceInformationPtr si_;
    ompl::base::StateSamplerPtr sampler_pos_;
    ompl::base::StateSamplerPtr sampler_alpha_;
    ompl::base::StateSamplerPtr sampler_formations_;

    static std::vector<ompl::base::State *> valid_formations_;

    bool uniform_;

};