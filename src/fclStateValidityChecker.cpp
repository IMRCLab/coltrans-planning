#include "fclStateValidityChecker.h"
#include <fcl/fcl.h>

fclStateValidityChecker::fclStateValidityChecker(
    ompl::base::SpaceInformationPtr si,
    std::shared_ptr<RobotsWithPayload> robots,
    std::shared_ptr<Obstacles> obstacles,
    const Eigen::VectorXf& attachmentpoints,
    const std::vector<double>& cablelengthVec,
    const std::string& payloadShape)
    
    : ompl::base::StateValidityChecker(si)
    , robots_(robots)
    , obstacles_(obstacles)
    , attachmentpoints_(attachmentpoints)
    , cablelengthVec_(cablelengthVec)
    , payloadShape_(payloadShape)
{

}

bool fclStateValidityChecker::isValid(const ompl::base::State* state) const
{
  if (!si_->satisfiesBounds(state)) {
    return false;
  }
  
  robots_->setPayloadTransformation(state);
  auto cableNums = si_->getStateSpace()->as<StateSpace>()->getNumCables();
  for(size_t i = 0; i<cableNums; ++i) {
    double length = cablelengthVec_[i];
    Eigen::Vector3f attachmentpoint(attachmentpoints_(0+3*i), attachmentpoints_(1+3*i), attachmentpoints_(2+3*i));
    // robots_->setCableTransformation(state, i, attachmentpoint, length);
    robots_->setUAVTransformation(state, i, attachmentpoint, length);
  }
  robots_->col_mgr_all->update();
  
  fcl::DefaultCollisionData<float> collision_data;
  robots_->col_mgr_all->collide(obstacles_->obsmanager, &collision_data, fcl::DefaultCollisionFunction<float>);
  // Inter-robot collision is still not implemented
  fcl::DefaultCollisionData<float> collision_data_robot;
  robots_->col_mgr_all->collide(&collision_data_robot, fcl::DefaultCollisionFunction<float>);
  
  if (collision_data.result.isCollision() || collision_data_robot.result.isCollision()) {
    return false;
  }

  return true;
}
