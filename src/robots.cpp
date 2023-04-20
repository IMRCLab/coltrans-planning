
#include "robots.h" 
namespace ob = ompl::base;
namespace ob = ompl::base;

RobotsWithPayload::RobotsWithPayload(const plannerSettings& cfg)
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

    // collision managers
    addRobotParts(cfg);
    col_mgr_all.reset(new fcl::DynamicAABBTreeCollisionManagerf());
    col_mgr_all->registerObject(payloadObj);
    col_mgr_all->registerObjects(uavObj);
    col_mgr_all->setup();

    col_mgr_cables.reset(new fcl::DynamicAABBTreeCollisionManagerf());
    col_mgr_cables->registerObjects(cablesObj);
    col_mgr_cables->setup();

}

void RobotsWithPayload::addRobotParts(const plannerSettings& cfg)
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
        uavgeom.reset(new fcl::Spheref(0.1));
        auto uavco = new fcl::CollisionObjectf(uavgeom);
        uavco->setTranslation(fcl::Vector3f(0,0,0));
        uavco->computeAABB();
        uavObj.push_back(uavco);
    }

}


fcl::Transform3f RobotsWithPayload::getPayloadTransform(const ob::State *state, const std::string& payloadShape)
{
    auto st = state->as<StateSpace::StateType>();
    fcl::Transform3f transform;
    Eigen::Vector3f pos(st->getPayloadPos());
    transform = Eigen::Translation<float, 3>(fcl::Vector3f(pos(0), pos(1), pos(2)));
    Eigen::Quaternionf payload_quat(st->getPayloadquat());
    if (payloadShape == "rod") {
        const Eigen::Quaternionf fclInmeshcat(0.7071068, -0.7071068, 0, 0);
        Eigen::Matrix3f payload_rotmat = fclInmeshcat.normalized().toRotationMatrix() * payload_quat.normalized().toRotationMatrix();
        Eigen::Quaternionf payload_quat_in_fcl(payload_rotmat);
        transform.rotate(payload_quat_in_fcl);
        return transform;
    }
    transform.rotate(payload_quat);
    return transform;
}

fcl::Transform3f RobotsWithPayload::getCableTransform(const ob::State *state, const size_t cableNum, Eigen::Vector3f& attachmentPoint,const double length)
{
    auto st = state->as<StateSpace::StateType>();
    fcl::Transform3f transform;
    Eigen::Vector3f attpoint = st->getAttPointInFixedFrame(attachmentPoint);
    Eigen::Quaternionf cablequat = st->getCableQuat(cableNum);
    Eigen::Vector3f rotatez = cablequat*fcl::Vector3f(0,0,0.5*length);
    transform = Eigen::Translation<float, 3>(attpoint + rotatez);
    transform.rotate(cablequat.toRotationMatrix());
    return transform;
}

fcl::Transform3f RobotsWithPayload::getUAVTransform(const ob::State *state, const size_t uavNum, Eigen::Vector3f& attachmentPoint,const double length)
{
    auto st = state->as<StateSpace::StateType>();
    fcl::Transform3f transform;
    Eigen::Vector3f uavPos = st->getuavPos(uavNum, attachmentPoint, length);
    Eigen::Quaternionf uavquat(1,0,0,0);
    transform = Eigen::Translation<float, 3>(fcl::Vector3f(uavPos(0), uavPos(1), uavPos(2)));
    transform.rotate(uavquat);
    return transform;
}

void RobotsWithPayload::setPayloadTransformation(const ob::State* state, const std::string& payloadShape)
{
    const auto& transform = getPayloadTransform(state, payloadShape);
    payloadObj->setTranslation(transform.translation());
    payloadObj->setRotation(transform.rotation());
    payloadObj->computeAABB();
}

void RobotsWithPayload::setCableTransformation(const ob::State* state, const size_t cableNum, Eigen::Vector3f& attachmentPoint,const double length)
{
    const auto& transform = getCableTransform(state, cableNum, attachmentPoint, length);
    cablesObj[cableNum]->setTranslation(transform.translation());
    cablesObj[cableNum]->setRotation(transform.rotation());
    cablesObj[cableNum]->computeAABB();
}   

void RobotsWithPayload::setUAVTransformation(const ob::State* state, const size_t uavNum, Eigen::Vector3f& attachmentPoint,const double length)
{
    const auto& transform = getUAVTransform(state, uavNum, attachmentPoint, length);
    uavObj[uavNum]->setTranslation(transform.translation());
    uavObj[uavNum]->setRotation(transform.rotation());
    uavObj[uavNum]->computeAABB();
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
//////////////////////////////////////////////////////////////////////////////////

const Eigen::Vector3f StateSpace::StateType::getPayloadPos() const
{
    auto pos =  as<ob::RealVectorStateSpace::StateType>(0)->values;
    Eigen::Vector3f payloadPos;
    for(size_t i = 0; i < 3; ++i) 
    {
        payloadPos(i) = pos[i];
    }
    return payloadPos;
}

void StateSpace::StateType::setPayloadPos(const Eigen::Vector3f& payloadPos) const
{
    auto pos =  as<ob::RealVectorStateSpace::StateType>(0)->values;
    pos[0] = payloadPos(0);
    pos[1] = payloadPos(1);
    pos[2] = payloadPos(2);
}
const Eigen::Quaternionf StateSpace::StateType::getPayloadquat() const 
{
    auto rot = as<ob::SO3StateSpace::StateType>(1);
    const Eigen::Quaternionf payload_quat(rot->w, rot->x, rot->y, rot->z);
    return payload_quat;
}
void StateSpace::StateType::setPayloadquat(const Eigen::Quaternionf& payload_quat)
{
    auto rot = as<ob::SO3StateSpace::StateType>(1);
    rot->w = payload_quat.w();
    rot->x = payload_quat.x();
    rot->y = payload_quat.y();
    rot->z = payload_quat.z();
} 
Eigen::Vector3f StateSpace::StateType::getCablePos(const size_t cableNum, Eigen::Vector3f& attachmentPoint,const double& length) const
{    
    Eigen::Vector3f unitvec = getunitvec(cableNum);
    Eigen::Vector3f attPointInFixedFrame = getAttPointInFixedFrame(attachmentPoint);
    return attPointInFixedFrame + (length/2)*unitvec;
}

Eigen::Quaternionf StateSpace::StateType::getCableQuat(size_t cableNum) const 
{      
    Eigen::Vector3f v1 = getunitvec(cableNum);
    Eigen::Vector3f v2(0,0,1);
    Eigen::Quaternionf q_rotation = Eigen::Quaternionf::FromTwoVectors(v2, v1);
    return q_rotation;
}

Eigen::Vector3f StateSpace::StateType::getuavPos(const size_t& cableNum, Eigen::Vector3f& attachmentPoint,const double& length) const
{    
    Eigen::Vector3f unitvec = getunitvec(cableNum);
    Eigen::Vector3f attPointInFixedFrame = getAttPointInFixedFrame(attachmentPoint);
    return attPointInFixedFrame + (length)*unitvec;
}

Eigen::Vector3f StateSpace::StateType::getunitvec(size_t cablenum) const
{    
    auto az = as<ob::RealVectorStateSpace::StateType>(2)->values[0+2*cablenum];
    auto el = as<ob::RealVectorStateSpace::StateType>(2)->values[1+2*cablenum];
    // azimuth and elevation --> unit vec
    // source https://math.stackexchange.com/questions/1150232/finding-the-unit-direction-vector-given-azimuth-and-elevation
    Eigen::Vector3f unitvec(cos(az)*cos(el), sin(az)*cos(el), sin(el));    
    return unitvec;
}

Eigen::Vector3f StateSpace::StateType::getAttPointInFixedFrame(Eigen::Vector3f& attachmentPoint) const
{    
    Eigen::Vector3f payloadPos = getPayloadPos();
    Eigen::Quaternionf payload_quat = getPayloadquat();

    Eigen::Vector3f attPointInFixedFrame = payloadPos + payload_quat.normalized() * attachmentPoint;
    return attPointInFixedFrame;
}


StateSpace::StateSpace(size_t numCables)
{
    setName("Quadrotor" + getName());
    type_ = ob::STATE_SPACE_TYPE_COUNT + 0;
    addSubspace(std::make_shared<ob::RealVectorStateSpace>(3), 1.0);      // position
    addSubspace(std::make_shared<ob::SO3StateSpace>(), 1.0);              // orientation
    addSubspace(std::make_shared<ob::RealVectorStateSpace>(2*numCables), 1.0); // cable az and el
    lock();
}

size_t StateSpace::getNumCables() const
{
    return as<ob::RealVectorStateSpace>(2)->getDimension() / 2;
}

void StateSpace::setPositionBounds(const ob::RealVectorBounds &bounds)
{
    as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
}

void StateSpace::setCableBounds(const ob::RealVectorBounds &bounds) 
{
    as<ob::RealVectorStateSpace>(2)->setBounds(bounds);
}

const ob::RealVectorBounds& StateSpace::getPositionBounds() const
{
    return as<ob::RealVectorStateSpace>(0)->getBounds();
}

ob::State* StateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

void StateSpace::freeState(ob::State *state) const
{
    CompoundStateSpace::freeState(state);
}

void StateSpace::registerProjections()
{
    class SE3DefaultProjection : public ob::ProjectionEvaluator
    {
    public:
    SE3DefaultProjection(const StateSpace *space) : ob::ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        return 3;
    }

    void defaultCellSizes() override
    {
        cellSizes_.resize(3);
        bounds_ = space_->as<StateSpace>()->getPositionBounds();
        cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
        cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
        cellSizes_[2] = (bounds_.high[2] - bounds_.low[2]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
    }

    void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        projection = Eigen::Map<const Eigen::VectorXd>(
            state->as<StateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values, 3);
    }
    };

    registerDefaultProjection(std::make_shared<SE3DefaultProjection>(this));
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<RobotsWithPayload> create_robots(const plannerSettings& cfg)
{
    std::shared_ptr<RobotsWithPayload> cableRobotSys;
    cableRobotSys.reset(new RobotsWithPayload(cfg));
    return cableRobotSys;
}

std::shared_ptr<Obstacles> create_obs(const YAML::Node &env)
{
    std::shared_ptr<Obstacles> obstacles;
    obstacles.reset(new Obstacles(env));
    return obstacles;
}
