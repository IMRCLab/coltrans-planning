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
    robots_->setUAVTransformation(state, i, attachmentpoint, length);
    robots_->setCableTransformation(state, i, attachmentpoint, length);
  }
  robots_->col_mgr_all->update();
  robots_->col_mgr_cables->update();
  // robot-obstacle collision 
  fcl::DefaultCollisionData<float> collision_data_robot_obs;
  robots_->col_mgr_all->collide(obstacles_->obsmanager, &collision_data_robot_obs, fcl::DefaultCollisionFunction<float>);
  // Inter-robot collision 
  fcl::DefaultCollisionData<float> collision_data_robots;
  robots_->col_mgr_all->collide(&collision_data_robots, fcl::DefaultCollisionFunction<float>);
  
  // cables/obstacles collision
  fcl::DefaultCollisionData<float> collision_data_cables_obs;
  robots_->col_mgr_cables->collide(obstacles_->obsmanager, &collision_data_cables_obs, fcl::DefaultCollisionFunction<float>);
  
  // inter-cable collision
  fcl::DefaultCollisionData<float> collision_data_cables;
  robots_->col_mgr_cables->collide(&collision_data_cables, fcl::DefaultCollisionFunction<float>);

  auto st = state->as<StateSpace::StateType>();
  auto payload_quat = st->getPayloadquat();
  Eigen::Vector3f e3(0,0,1);
  Eigen::Vector3f v(qvrot(payload_quat, e3));
  auto angle = acosf(e3.dot(v));

  if (cfg_.payloadShape == "triangle" || cfg_.payloadShape == "rod") {
    if (angle > cfg_.angle_max) {
      return false;
    } else if (angle < cfg_.angle_min) {
      return false;
    }
  }
  if (collision_data_robots.result.isCollision() || collision_data_robot_obs.result.isCollision()  || collision_data_cables.result.isCollision() || collision_data_cables_obs.result.isCollision()) {
    return false;
  }

  return true;
}



fclUnicyclesStateValidityChecker::fclUnicyclesStateValidityChecker(
    ompl::base::SpaceInformationPtr si,
    std::shared_ptr<UnicyclesWithRods> robots,
    std::shared_ptr<Obstacles> obstacles,
    const unicycleSettings& cfg)
    
    : ompl::base::StateValidityChecker(si)
    , robots_(robots)
    , obstacles_(obstacles)
    , cfg_(cfg)
{

}


bool fclUnicyclesStateValidityChecker::isValid(const ompl::base::State* state) const
{
  if (!si_->satisfiesBounds(state)) {
    return false;
  }

  auto num_robots = si_->getStateSpace()->as<UnicyclesStateSpace>()->getNumRobots();

  for(size_t i=0; i<num_robots;++i) {
    robots_->setRobotTransform(state, i);
  }

  for(size_t i=0; i<num_robots-1; ++i) {
    robots_->setRodTransform(state, i);
  } 

  robots_->col_mgr_robots->update();
  robots_->col_mgr_rods->update();


  fcl::DefaultCollisionData<float> collision_data_robot_obs;
  robots_->col_mgr_robots->collide(obstacles_->obsmanager, &collision_data_robot_obs, fcl::DefaultCollisionFunction<float>);

  fcl::DefaultCollisionData<float> collision_data_robots;
  robots_->col_mgr_robots->collide(&collision_data_robots, fcl::DefaultCollisionFunction<float>);

  fcl::DefaultCollisionData<float> collision_data_rod_obs;
  robots_->col_mgr_rods->collide(obstacles_->obsmanager, &collision_data_rod_obs, fcl::DefaultCollisionFunction<float>);

  fcl::DefaultCollisionData<float> collision_data_rod_rod;
  robots_->col_mgr_rods->collide(&collision_data_rod_rod, fcl::DefaultCollisionFunction<float>);


  if (collision_data_robots.result.isCollision() || collision_data_robot_obs.result.isCollision()  || collision_data_rod_obs.result.isCollision() || collision_data_rod_rod.result.isCollision()) {
    return false;
  }


  return true;
}
