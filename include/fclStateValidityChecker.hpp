#pragma once

// #include "environment.h"
// #include "robots.h"

#include <fcl/fcl.h>
#include <robots.hpp>

class fclStateValidityChecker
  : public ompl::base::StateValidityChecker
{
public:
  fclStateValidityChecker(
      ompl::base::SpaceInformationPtr si,
      std::shared_ptr<nCablesPayload> robots,
      std::shared_ptr<Obstacles> obstacles,
      const Eigen::VectorXf& attachmentpoints,
      const std::vector<double>& cablelengthVec)
      
      : ompl::base::StateValidityChecker(si)
      , robots_(robots)
      , obstacles_(obstacles)
      , attachmentpoints_(attachmentpoints)
      , cablelengthVec_(cablelengthVec)
  {
  }

  bool isValid(const ompl::base::State* state) const override
  {
    if (!si_->satisfiesBounds(state)) {
      return false;
    }
    
    robots_->sysparts = new fcl::DynamicAABBTreeCollisionManagerf();
    robots_->setPayloadTransformation(state);
    auto cableNums = si_->getStateSpace()->as<StateSpace>()->getNumCables();
    for(size_t i = 0; i<cableNums; ++i) {
      double length = cablelengthVec_[i];
      Eigen::Vector3f attachmentpoint(attachmentpoints_(0+3*i), attachmentpoints_(1+3*i), attachmentpoints_(2+3*i));
      robots_->setCableTransformation(state, i, attachmentpoint, length);
      robots_->setUAVTransformation(state, i, attachmentpoint, length);
    }
    robots_->setSysParts();
   
    fcl::DefaultCollisionData<float> collision_data;
    robots_->sysparts->collide(obstacles_->obsmanager, &collision_data, fcl::DefaultCollisionFunction<float>);
    // Inter-robot collision is still not implemented
    // robots_->sysparts->collide(&collision_data, fcl::DefaultCollisionFunction<float>);
    if (collision_data.result.isCollision()) {
      return false;
    }
    return true;
  }

private:
  std::shared_ptr<nCablesPayload> robots_;
  std::shared_ptr<Obstacles> obstacles_;
  const Eigen::VectorXf attachmentpoints_;
  std::vector<double> cablelengthVec_;
};
