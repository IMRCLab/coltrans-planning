
#include <robots.hpp> 
namespace ob = ompl::base;
namespace ob = ompl::base;

nCablesPayload::nCablesPayload(const plannerSettings& cfg)
{
    // SE3 for the payload
    auto payloadPos(std::make_shared<ob::RealVectorStateSpace>(3));
    auto payloadRot(std::make_shared<ob::SO3StateSpace>());
    // orientation of the two cables: az1, elv1, az2, elv2
    auto cables(std::make_shared<ob::RealVectorStateSpace>(2*cfg.numofcables));

    ob::RealVectorBounds payloadbounds(3);
    // prepare the payload position bounds
    for (size_t i=0; i < cfg.envmin.size(); ++i) {
        payloadbounds.setLow(i, cfg.envmin[i]);
        payloadbounds.setHigh(i, cfg.envmax[i]);
    }
    payloadPos->setBounds(payloadbounds);

    ob::RealVectorBounds cablebounds(2*cfg.numofcables);
    // prepare the cable elevation bounds
    for (size_t i=0; i < cfg.cablemin.size(); ++i) 
    {
        cablebounds.setLow(i, cfg.cablemin[i]);
        cablebounds.setHigh(i, cfg.cablemax[i]);
    }
    cables->setBounds(cablebounds);
    auto space = payloadPos +payloadRot + cables;

    // construct an instance of space information from this state
    si = std::make_shared<ob::SpaceInformation>(space);
    std::cout << "state space dimension : " <<si->getStateDimension()<<std::endl;
    // setup the state space information
    si->setup();


}

void nCablesPayload::addRobotParts(const plannerSettings& cfg)
{
    // parts are: n cables, n spheres for uavs, payload = 2*n+1
    
    // add cables and uavs
    for (size_t i = 0; i < cfg.numofcables; ++i) 
    {
        std::shared_ptr<fcl::CollisionGeometryf> cablegeom;
        cablegeom.reset(new fcl::Cylinderf(0.001, cfg.cableslength[i]));
        auto cableco = new fcl::CollisionObjectf(cablegeom);
        cableco->setTranslation(fcl::Vector3f(0,0,0));
        cableco->computeAABB();
        systemParts.push_back(cableco);
        
        std::shared_ptr<fcl::CollisionGeometryf> uavgeom;
        uavgeom.reset(new fcl::Spheref(0.15));
        auto uavco = new fcl::CollisionObjectf(uavgeom);
        uavco->setTranslation(fcl::Vector3f(0,0,0));
        uavco->computeAABB();
        systemParts.push_back(uavco);
    }

    // add payload
    std::shared_ptr<fcl::CollisionGeometryf> payloadgeom;
    if (cfg.payloadShape == "rod")
    {
        payloadgeom.reset(new fcl::Cylinderf(0.025, 0.2));
    } else if (cfg.payloadShape == " Triangle")
    {
        payloadgeom.reset(new fcl::Boxf(0.08, 0.08, 0.005));
    }
    else {
        payloadgeom.reset(new fcl::Spheref(0.01));
    }
    auto payloadco = new fcl::CollisionObjectf(payloadgeom);
    payloadco->setTranslation(fcl::Vector3f(0,0,0));
    payloadco->setQuatRotation(fcl::Quaternionf(1,0,0,0));
    payloadco->computeAABB();
    systemParts.push_back(payloadco);

    systemParts.push_back(payloadco);
    sysparts = new fcl::DynamicAABBTreeCollisionManagerf();
    sysparts->registerObjects(systemParts);
    sysparts->setup();
}


Obstacles::Obstacles(const YAML::Node &env)
{
     for (const auto &obs : env)
  {
    if (obs["type"].as<std::string>() == "sphere")
    {
      const auto &radius = obs["radius"];
      std::shared_ptr<fcl::CollisionGeometryf> geom;
      geom.reset(new fcl::Spheref(radius.as<float>()));
      const auto &center = obs["center"];
      auto co = new fcl::CollisionObjectf(geom);
      co->setTranslation(fcl::Vector3f(center[0].as<float>(), center[1].as<float>(), center[2].as<float>()));
      co->computeAABB();
      obstacles.push_back(co);
    }
  }
  obsmanager = new fcl::DynamicAABBTreeCollisionManagerf();
  obsmanager->registerObjects(obstacles);
  obsmanager->setup();
}

// void obstacles::addObstacles(const YAML::Node &env)
// {

// }