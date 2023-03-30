#include "fclStateValidityChecker.h"
#include <fcl/fcl.h>

fclStateValidityChecker::fclStateValidityChecker(
    ompl::base::SpaceInformationPtr si,
    std::shared_ptr<RobotsWithPayload> robots,
    std::shared_ptr<Obstacles> obstacles,
    const plannerSettings& cfg)
    
    : ompl::base::StateValidityChecker(si)
    , robots_(robots)
    , obstacles_(obstacles)
    , cfg_(cfg)
{

}

Eigen::Vector3f fclStateValidityChecker::qvrot(const Eigen::Quaternionf &q, const Eigen::Vector3f &v) const
{
  Eigen::Quaternionf p;
  p.w() = 0;
  p.vec() = v;
  Eigen::Quaternionf v_rotated = q * p * q.inverse(); 
  return  v_rotated.vec();
}   


bool fclStateValidityChecker::isValid(const ompl::base::State* state) const
{
  if (!si_->satisfiesBounds(state)) {
    return false;
  }
  
  robots_->setPayloadTransformation(state, cfg_.payloadShape);
  auto cableNums = si_->getStateSpace()->as<StateSpace>()->getNumCables();
  for(size_t i = 0; i<cableNums; ++i) {
    double length = cfg_.cablelengthVec[i];
    Eigen::Vector3f attachmentpoint(cfg_.attachmentpoints(0+3*i), cfg_.attachmentpoints(1+3*i), cfg_.attachmentpoints(2+3*i));
    // robots_->setCableTransformation(state, i, attachmentpoint, length);
    robots_->setUAVTransformation(state, i, attachmentpoint, length);
  }
  robots_->col_mgr_all->update();
  
  fcl::DefaultCollisionData<float> collision_data;
  robots_->col_mgr_all->collide(obstacles_->obsmanager, &collision_data, fcl::DefaultCollisionFunction<float>);
  // Inter-robot collision is still not implemented
  fcl::DefaultCollisionData<float> collision_data_robot;
  robots_->col_mgr_all->collide(&collision_data_robot, fcl::DefaultCollisionFunction<float>);
  

  auto angle_min = cfg_.angle_min;
  auto angle_max = cfg_.angle_max;
  auto st = state->as<StateSpace::StateType>();
  auto payload_quat = st->getPayloadquat();
  Eigen::Vector3f e3(0,0,1);
  Eigen::Vector3f v(qvrot(payload_quat, e3));
  auto angle = acosf(e3.dot(v));
  
  if (angle > angle_max) {
    return false;
  } else if (angle < angle_min) {
    return false;
  }
  if (collision_data.result.isCollision() || collision_data_robot.result.isCollision()) {
    return false;
  }

  return true;
}
