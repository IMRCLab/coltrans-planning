
#include <robots.hpp> 
namespace ob = ompl::base;
namespace ob = ompl::base;

nCablesPayload::nCablesPayload(const plannerSettings& cfg)
{
    // SE3 for the payload
    auto space(std::make_shared<StateSpace>(cfg.numofcables));

    ob::RealVectorBounds payloadbounds(3);
    // prepare the payload position bounds
    for (size_t i=0; i < cfg.envmin.size(); ++i) {
        payloadbounds.setLow(i, cfg.envmin[i]);
        payloadbounds.setHigh(i, cfg.envmax[i]);
    }
    space->setPositionBounds(payloadbounds);

    ob::RealVectorBounds cablebounds(2*cfg.numofcables);
    // prepare the cable elevation bounds
    for (size_t i=0; i < cfg.cablemin.size(); ++i) 
    {
        cablebounds.setLow(i, cfg.cablemin[i]);
        cablebounds.setHigh(i, cfg.cablemax[i]);
    }
    space->setCableBounds(cablebounds);

    // construct an instance of space information from this state
    si = std::make_shared<ob::SpaceInformation>(space);
    std::cout << "state space dimension : " <<si->getStateDimension()<<std::endl;
    // setup the state space information
    si->setup();
}

void nCablesPayload::addRobotParts(const plannerSettings& cfg)
{
    // add payload
      std::shared_ptr<fcl::CollisionGeometryf> geom;
    if (cfg.payloadShape == "rod")
    {
        geom.reset(new fcl::Cylinderf(0.025, 0.2));
    } else if (cfg.payloadShape == " Triangle")
    {
        geom.reset(new fcl::Boxf(0.08, 0.08, 0.005));
    }
    else {
        geom.reset(new fcl::Spheref(0.01));
    }
    payloadObj = new fcl::CollisionObjectf(geom);

    // parts are: n cables, n spheres for uavs, payload = 2*n+1
    // add cables and uavs
    for (size_t i = 0; i < cfg.numofcables; ++i) 
    {
        std::shared_ptr<fcl::CollisionGeometryf> cablegeom;
        cablegeom.reset(new fcl::Cylinderf(0.001, cfg.cablelengthVec[i]));
        auto cableco = new fcl::CollisionObjectf(cablegeom);
        cableco->setTranslation(fcl::Vector3f(0,0,0));
        cableco->computeAABB();
        cablesObj.push_back(cableco);

        std::shared_ptr<fcl::CollisionGeometryf> uavgeom;
        uavgeom.reset(new fcl::Spheref(0.15));
        auto uavco = new fcl::CollisionObjectf(uavgeom);
        uavco->setTranslation(fcl::Vector3f(0,0,0));
        uavco->computeAABB();
        uavObj.push_back(uavco);
    }

}


fcl::Transform3f nCablesPayload::getPayloadTransform(const ob::State *state)
{
    auto st = state->as<StateSpace::StateType>();
    fcl::Transform3f transform;
    Eigen::Vector3f pos = st->getPayloadPos();
    transform = Eigen::Translation<float, 3>(fcl::Vector3f(pos(0), pos(1), pos(2)));
    transform.rotate(st->getPayloadquat());
    return transform;
}

fcl::Transform3f nCablesPayload::getCableTransform(const ob::State *state, const size_t cableNum, Eigen::Vector3f& attachmentPoint,const double length)
{
    auto st = state->as<StateSpace::StateType>();
    fcl::Transform3f transform;
    Eigen::Vector3f cablePos = st->getCablePos(cableNum, attachmentPoint, length);
    Eigen::Quaternionf cablequat = st->getCableQuat(cableNum);
    transform = Eigen::Translation<float, 3>(fcl::Vector3f(cablePos(0), cablePos(1), cablePos(2)));
    transform.rotate(cablequat);
    return transform;
}

fcl::Transform3f nCablesPayload::getUAVTransform(const ob::State *state, const size_t uavNum, Eigen::Vector3f& attachmentPoint,const double length)
{
    auto st = state->as<StateSpace::StateType>();
    fcl::Transform3f transform;
    Eigen::Vector3f uavPos = st->getuavPos(uavNum, attachmentPoint, length);
    Eigen::Quaternionf uavquat(1,0,0,0);
    transform = Eigen::Translation<float, 3>(fcl::Vector3f(uavPos(0), uavPos(1), uavPos(2)));
    transform.rotate(uavquat);
    return transform;
}

void nCablesPayload::setPayloadTransformation(const ob::State* state)
{
    const auto& transform = getPayloadTransform(state);
    payloadObj->setTranslation(transform.translation());
    payloadObj->setRotation(transform.rotation());
    payloadObj->computeAABB();
}

void nCablesPayload::setCableTransformation(const ob::State* state, const size_t cableNum, Eigen::Vector3f& attachmentPoint,const double length)
{
    const auto& transform = getCableTransform(state, cableNum, attachmentPoint, length);
    cablesObj[cableNum]->setTranslation(transform.translation());
    cablesObj[cableNum]->setRotation(transform.rotation());
    cablesObj[cableNum]->computeAABB();
}   

void nCablesPayload::setUAVTransformation(const ob::State* state, const size_t uavNum, Eigen::Vector3f& attachmentPoint,const double length)
{
    const auto& transform = getUAVTransform(state, uavNum, attachmentPoint, length);
    uavObj[uavNum]->setTranslation(transform.translation());
    uavObj[uavNum]->setRotation(transform.rotation());
    uavObj[uavNum]->computeAABB();
}

void nCablesPayload::setSysParts() {
    sysparts->registerObject(payloadObj);
    sysparts->registerObjects(cablesObj);
    sysparts->registerObjects(uavObj);
    sysparts->setup();


}
Obstacles::Obstacles(const YAML::Node &env)
{
    for (const auto &obs : env)
    {
    if (obs["type"].as<std::string>() == "sphere")
    {
        std::shared_ptr<fcl::CollisionGeometryf> geom;
        const auto &radius = obs["radius"];
        geom.reset(new fcl::Spheref(radius.as<float>()));      
        const auto &center = obs["center"];
        auto co = new fcl::CollisionObjectf(geom);
        co->setTranslation(fcl::Vector3f(center[0].as<float>(), center[1].as<float>(), center[2].as<float>()));
        co->computeAABB();
        obstacles.push_back(co);    
    } else if (obs["type"].as<std::string>() == "cylinder")
    {
        std::shared_ptr<fcl::CollisionGeometryf> geom;
        const auto &radius = obs["radius"];
        const auto &height = obs["height"];
        geom.reset(new fcl::Cylinderf(radius.as<float>(), height.as<float>()));
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

size_t Obstacles::getObsNum()
{
    return obstacles.size();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<nCablesPayload> create_sys(const plannerSettings& cfg)
{
    std::shared_ptr<nCablesPayload> cableRobotSys;
    cableRobotSys.reset(new nCablesPayload(cfg));
    return cableRobotSys;
}

std::shared_ptr<Obstacles> create_obs(const YAML::Node &env)
{
    std::shared_ptr<Obstacles> obstacles;
    obstacles.reset(new Obstacles(env));
    return obstacles;
}
