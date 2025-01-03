
#include "robots.h" 
#include "fclStateValidityChecker.h"
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

    if (cfg.numofcables > 0) {
        ob::RealVectorBounds cablebounds(2*cfg.numofcables);
        // prepare the cable elevation bounds
        for (size_t i=0; i < cfg.cablemin.size(); ++i) 
        {
            cablebounds.setLow(i, cfg.cablemin[i]);
            cablebounds.setHigh(i, cfg.cablemax[i]);
        }
        space->setCableBounds(cablebounds);
    }

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
        geom.reset(new fcl::Boxf(0.025, 0.6, 0.025));
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
        // std::shared_ptr<fcl::CollisionGeometryf> cablegeom;
        // cablegeom.reset(new fcl::Cylinderf(0.001, cfg.cablelengthVec[i] - 0.01));
        std::shared_ptr<fcl::CollisionGeometryf> cablegeom;
        cablegeom.reset(new fcl::Capsulef(0.01, 0.2*cfg.cablelengthVec[i]));
        auto cableco = new fcl::CollisionObjectf(cablegeom);
        cableco->setTranslation(fcl::Vector3f(0,0,0));
        cableco->computeAABB();
        cablesObj.push_back(cableco);

        std::shared_ptr<fcl::CollisionGeometryf> uavgeom;
        uavgeom.reset(new fcl::Spheref(cfg.robot_radius));
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
    // if (payloadShape == "rod") {
    //     const Eigen::Quaternionf fclInmeshcat(0.7071068, -0.7071068, 0, 0);
    //     Eigen::Matrix3f payload_rotmat = fclInmeshcat.normalized().toRotationMatrix() * payload_quat.normalized().toRotationMatrix();
    //     Eigen::Quaternionf payload_quat_in_fcl(payload_rotmat);
    //     transform.rotate(payload_quat_in_fcl);
    //     return transform;
    // }
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

Obstacles::Obstacles()
{
    obsmanager = new fcl::DynamicAABBTreeCollisionManagerf();
    obsmanager->setup();
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
    } else if (obs["type"].as<std::string>() == "box") {
        std::shared_ptr<fcl::CollisionGeometryf> geom;
        const auto &size = obs["size"];
        geom.reset(new fcl::Boxf(size[0].as<float>(), size[1].as<float>(), size[2].as<float>()));
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

Obstacles::~Obstacles()
{
    delete obsmanager;
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


StateSpace::StateSpace(size_t numCables, double cable_weight)
{
    setName("Quadrotor" + getName());
    type_ = ob::STATE_SPACE_TYPE_COUNT + 0;
    num_cables_ = numCables;
    addSubspace(std::make_shared<ob::RealVectorStateSpace>(3), 1.0);      // position
    addSubspace(std::make_shared<ob::SO3StateSpace>(), 1.0);              // orientation
    if (numCables > 0) {
        addSubspace(std::make_shared<ob::RealVectorStateSpace>(2*numCables), cable_weight); // cable az and el
    }
    lock();
}

size_t StateSpace::getNumCables() const
{
    return num_cables_;
}

void StateSpace::setPositionBounds(const ob::RealVectorBounds &bounds)
{
    as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
}

const ob::RealVectorBounds& StateSpace::getPositionBounds() const
{
    return as<ob::RealVectorStateSpace>(0)->getBounds();
}

void StateSpace::setCableBounds(const ob::RealVectorBounds &bounds) 
{
    as<ob::RealVectorStateSpace>(2)->setBounds(bounds);
}

const ob::RealVectorBounds& StateSpace::getCableBounds() const
{
    return as<ob::RealVectorStateSpace>(2)->getBounds();
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

/////////////////////// StateSpace Unicycles with Rods ///////////////////////////

UnicyclesWithRods::UnicyclesWithRods(const unicycleSettings& cfg) {
    // px py alpha1 alpha2 alph3 th1 th2 (3 robots)
    auto space(std::make_shared<UnicyclesStateSpace>(cfg.num_robots, cfg.envmin, cfg.envmax));

    ob::RealVectorBounds envBounds(2);
    for (size_t i=0; i < cfg.envmin.size(); ++i) {
        envBounds.setLow(i, cfg.envmin[i]);
        envBounds.setHigh(i, cfg.envmax[i]);
    }
    space->setPositionBounds(envBounds);
    ob::RealVectorBounds rotationBounds(cfg.num_robots); //alpha_i, ..., alpha_n

    for (size_t i=0; i < cfg.num_robots; ++i) {
        rotationBounds.setLow(i, -M_PI);
        rotationBounds.setHigh(i, M_PI);
    }
    space->setRotationBounds(rotationBounds);

    ob::RealVectorBounds rodBounds(cfg.num_robots - 1); //th_i, ..., th_(n-1) 
    for (size_t i=0; i < cfg.num_robots-1; ++i) {
        rodBounds.setLow(i, -M_PI);
        rodBounds.setHigh(i, M_PI);
    }
    space->setRodBounds(rodBounds);


    // construct an instance of space information from this state
    si = std::make_shared<ob::SpaceInformation>(space);
    std::cout << "state space dimension : " <<si->getStateDimension()<<std::endl;
    // setup the state space information
    si->setup();

    // collision managers
    addRobotParts(cfg);
    col_mgr_robots.reset(new fcl::DynamicAABBTreeCollisionManagerf());
    col_mgr_robots->registerObjects(robotsObj);
    col_mgr_robots->setup();

    col_mgr_rods.reset(new fcl::DynamicAABBTreeCollisionManagerf());
    col_mgr_rods->registerObjects(rodsObj);
    col_mgr_rods->setup();
}

//////////////////////// Unicycles with Rods system ////////////////////
void UnicyclesWithRods::addRobotParts(const unicycleSettings& cfg) {
    size_t num_robots = cfg.num_robots;
    // n robots
    for (size_t i=0; i < num_robots; ++i) {
      std::shared_ptr<fcl::CollisionGeometryf> geom;
        geom.reset(new fcl::Boxf(0.1, 0.05, 0.05));
        auto robotco = new fcl::CollisionObjectf(geom);
        robotco->setTranslation(fcl::Vector3f(0,0,0));
        robotco->computeAABB();
        robotsObj.push_back(robotco);
    }

    // n-1 rods
    for (size_t i=0; i < num_robots - 1; ++i) {
      std::shared_ptr<fcl::CollisionGeometryf> geom;
        geom.reset(new fcl::Boxf(0.6*0.5, 0.01, 0.01));
        auto rodco = new fcl::CollisionObjectf(geom);
        rodco->setTranslation(fcl::Vector3f(0,0,0));
        rodco->computeAABB();
        rodsObj.push_back(rodco);
    }
}

fcl::Transform3f UnicyclesWithRods::getRobotTransform(const ompl::base::State *state, const size_t idx) {
    auto st = state->as<UnicyclesStateSpace::StateType>();
    fcl::Transform3f transform;
    Eigen::Vector3f robotSt(st->getRobotSt(idx));
    transform = Eigen::Translation<float, 3>(fcl::Vector3f(robotSt(0), robotSt(1), 0.));
    transform.rotate(Eigen::AngleAxisf(robotSt(2), Eigen::Vector3f::UnitZ()));
    return transform;
}

fcl::Transform3f UnicyclesWithRods::getRodTransform(const ompl::base::State *state, const size_t idx) {
    auto st = state->as<UnicyclesStateSpace::StateType>();
    fcl::Transform3f transform;
    Eigen::Vector3f rodSt(st->getRodSt(idx));
    transform = Eigen::Translation<float, 3>(fcl::Vector3f(rodSt(0), rodSt(1), 0.));
    transform.rotate(Eigen::AngleAxisf(rodSt(2), Eigen::Vector3f::UnitZ()));
    return transform;
}

void UnicyclesWithRods::setRobotTransform(const ob::State *state, const size_t idx) {
    const auto& transform = getRobotTransform(state, idx);
    robotsObj[idx]->setTranslation(transform.translation());
    robotsObj[idx]->setRotation(transform.rotation());
    robotsObj[idx]->computeAABB();
}


void UnicyclesWithRods::setRodTransform(const ob::State *state, const size_t idx) {
    const auto& transform = getRodTransform(state, idx);
    rodsObj[idx]->setTranslation(transform.translation());
    rodsObj[idx]->setRotation(transform.rotation());
    rodsObj[idx]->computeAABB();
}

const Eigen::Vector3f UnicyclesStateSpace::StateType::getRobotSt(const size_t idx) const {
    double px = as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    double py = as<ob::RealVectorStateSpace::StateType>(0)->values[1];
    for (size_t i = 0; i < idx; ++i) {
        double theta = as<ob::RealVectorStateSpace::StateType>(2)->values[i];
        double length = 0.5;
        px += length * cos(theta);
        py += length * sin(theta);
    }
    double alpha = as<ob::RealVectorStateSpace::StateType>(1)->values[idx];
    return Eigen::Vector3f(px, py, alpha);
}

const Eigen::Vector3f UnicyclesStateSpace::StateType::getRodSt(const size_t idx) const {

    double px = as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    double py = as<ob::RealVectorStateSpace::StateType>(0)->values[1];
    double length = 0.5;    // Assuming fixed rod length

    for (size_t i = 0; i < idx; ++i) {
        double theta = as<ob::RealVectorStateSpace::StateType>(2)->values[i];
        px += length * cos(theta);
        py += length * sin(theta);
    }
    double theta_i = as<ob::RealVectorStateSpace::StateType>(2)->values[idx];
    double px_cable = px + 0.5*length*cos(theta_i);
    double py_cable = py + 0.5*length*sin(theta_i);
    return Eigen::Vector3f(px_cable, py_cable, theta_i);
}

UnicyclesStateSpace::UnicyclesStateSpace(size_t num_robots, std::vector<double> envmin, std::vector<double> envmax) {
    setName("Unicycle_" + getName());
    type_ = ob::STATE_SPACE_TYPE_COUNT + 0;
    num_robots_ = num_robots;
    env_min_ = envmin;
    env_max_ = envmax;
    addSubspace(std::make_shared<ob::RealVectorStateSpace>(2), 1.0);
    addSubspace(std::make_shared<ob::RealVectorStateSpace>(num_robots_), 0.1);
    addSubspace(std::make_shared<ob::RealVectorStateSpace>(num_robots_-1), 0.1);
    lock();
}

const ob::RealVectorBounds& UnicyclesStateSpace::getPositionBounds() const
{
    return as<ob::RealVectorStateSpace>(0)->getBounds();
}

void UnicyclesStateSpace::setPositionBounds(const ob::RealVectorBounds &bounds) {
    as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
}

void UnicyclesStateSpace::setRotationBounds(const ob::RealVectorBounds &bounds) {
    as<ob::RealVectorStateSpace>(1)->setBounds(bounds);
} 

void UnicyclesStateSpace::setRodBounds(const ob::RealVectorBounds &bounds) {
    as<ob::RealVectorStateSpace>(2)->setBounds(bounds);
}

bool UnicyclesStateSpace::satisfiesBounds(const ob::State *state) const {
    auto stateTyped = state->as<UnicyclesStateSpace::StateType>();
    std::vector<double> pxs;
    std::vector<double> pys;
    for (size_t i = 0; i < num_robots_; ++i) {
        Eigen::Vector3f robotst = stateTyped->getRobotSt(i);
        // std::cout << "robot i pos: " << i << ", " << robotst(0) << " , " << robotst(1) << std::endl;
        pxs.push_back(robotst(0));
        pys.push_back(robotst(1));
    }

    for (size_t i=0; i<pxs.size();++i) {    
        if (pxs[i] < env_min_[0] || pxs[i] > env_max_[0]) {
            return false; 
        }
    }
    for (size_t i=0; i<pys.size();++i) {
        if (pys[i] < env_min_[1] || pys[i] > env_max_[1]) {
            return false;
        }
    }
    return true; 
}

size_t UnicyclesStateSpace::getNumRobots() const
{
    return num_robots_;
}

ob::State* UnicyclesStateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

void UnicyclesStateSpace::freeState(ob::State *state) const
{
    CompoundStateSpace::freeState(state);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<RobotsWithPayload> create_robots(const plannerSettings& cfg)
{
    std::shared_ptr<RobotsWithPayload> cableRobotSys;
    cableRobotSys.reset(new RobotsWithPayload(cfg));
    return cableRobotSys;
}

std::shared_ptr<UnicyclesWithRods> create_robots(const unicycleSettings& cfg)
{
    std::shared_ptr<UnicyclesWithRods> unicyclesWithRods;
    unicyclesWithRods.reset(new UnicyclesWithRods(cfg));
    return unicyclesWithRods;
}

std::shared_ptr<Obstacles> create_obs(const YAML::Node &env)
{
    std::shared_ptr<Obstacles> obstacles;
    obstacles.reset(new Obstacles(env));
    return obstacles;
}
