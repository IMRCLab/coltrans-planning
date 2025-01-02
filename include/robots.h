#pragma once

// OMPL headers
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/tools/config/MagicConstants.h>

#include <fcl/fcl.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
namespace ob = ompl::base;


class Obstacles
{
public:
    Obstacles();
    Obstacles(const YAML::Node &env);
    ~Obstacles();
    std::vector<fcl::CollisionObjectf *> obstacles;
    fcl::BroadPhaseCollisionManagerf* obsmanager;
};

class unicycleSettings 
{
public:
    Eigen::VectorXf start;
    Eigen::VectorXf goal;
    YAML::Node env;
    std::vector<double> envmin;
    std::vector<double> envmax;
    size_t num_robots;
    float timelimit;
    std::vector<double> robot_size;
    float angle_min;
    float angle_max;
    float interpolate;
    std::string sampler;
};

// Class to set the environment and the robot settings 
class plannerSettings 
{
public:
    Eigen::VectorXf start;
    Eigen::VectorXf goal;
    YAML::Node env;
    Eigen::VectorXf attachmentpoints;
    std::vector<double> envmin;
    std::vector<double> envmax;
    std::vector<double> cablemin;
    std::vector<double> cablemax;
    std::vector<double> cablelengthVec;
    std::string plannerType;
    std::string payloadShape;
    std::string robot_type;
    size_t numofcables;
    float timelimit;
    float robot_radius;
    float angle_min;
    float angle_max;
    float interpolate;
    std::string sampler;
};

class RobotsWithPayload
{
public:
    RobotsWithPayload(const plannerSettings& cfg);


    fcl::Transform3f getPayloadTransform(const ompl::base::State *state, const std::string& payloadType);
    fcl::Transform3f getCableTransform(const ompl::base::State *state, const size_t cableNum, Eigen::Vector3f& attachmentPoint,const double length);
    fcl::Transform3f getUAVTransform(const ompl::base::State *state, const size_t cableNum, Eigen::Vector3f& attachmentPoint,const double length);
    
    void setPayloadTransformation(const ob::State *state, const std::string& payloadType);
    void setCableTransformation(const ob::State *state, const size_t cableNum, Eigen::Vector3f& attachmentPoint,const double length);
    void setUAVTransformation(const ob::State *state,   const size_t uavNum, Eigen::Vector3f& attachmentPoint,const double length);

    std::shared_ptr<ob::SpaceInformation> si;
    fcl::CollisionObjectf* payloadObj;
    std::vector<fcl::CollisionObjectf *> cablesObj;
    std::vector<fcl::CollisionObjectf *> uavObj;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mgr_all;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mgr_cables;

protected:
    void addRobotParts(const plannerSettings& cfg);
};

std::shared_ptr<RobotsWithPayload> create_robots(const plannerSettings& cfg);
std::shared_ptr<Obstacles> create_obs(const YAML::Node &env);

class StateSpace : public ob::CompoundStateSpace
{
public:
    class StateType : public ob::CompoundStateSpace::StateType
    {
    public:
        StateType() = default;

        const Eigen::Vector3f getPayloadPos() const;
        void setPayloadPos(const Eigen::Vector3f& payloadPos) const;
        const Eigen::Quaternionf getPayloadquat() const ;
        void setPayloadquat(const Eigen::Quaternionf& payload_quat);
        Eigen::Vector3f getCablePos(const size_t cableNum, Eigen::Vector3f& attachmentPoint,const double& length) const;
        Eigen::Quaternionf getCableQuat(size_t cableNum) const;
        Eigen::Vector3f getuavPos(const size_t& cableNum, Eigen::Vector3f& attachmentPoint,const double& length) const;
        Eigen::Vector3f getunitvec(size_t cablenum) const;
        Eigen::Vector3f getAttPointInFixedFrame(Eigen::Vector3f& attachmentPoint) const;
    };

    StateSpace(size_t numCables, double cable_weight = 1.0);
    ~StateSpace() override = default;

    size_t getNumCables() const;
    void setPositionBounds(const ob::RealVectorBounds &bounds);
    const ob::RealVectorBounds &getPositionBounds() const;
    
    void setCableBounds(const ob::RealVectorBounds &bounds);
    const ob::RealVectorBounds &getCableBounds() const;

    ob::State *allocState() const override;
    void freeState(ob::State *state) const override;
    void registerProjections();

protected:
    size_t num_cables_;

};


class UnicyclesWithRods
{
public:
    UnicyclesWithRods(const unicycleSettings& cfg);

    fcl::Transform3f getRobotTransform(const ompl::base::State *state, const size_t idx);
    fcl::Transform3f getRodTransform(const ompl::base::State *state, const size_t idx);
    
    void setRobotTransform(const ob::State *state, const size_t idx);
    void setRodTransform(const ob::State *state, const size_t idx);

    std::shared_ptr<ob::SpaceInformation> si;
    std::vector<fcl::CollisionObjectf *> robotsObj;
    std::vector<fcl::CollisionObjectf *> rodsObj;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mgr_robots;
    std::shared_ptr<fcl::BroadPhaseCollisionManagerf> col_mgr_rods;

protected:
    void addRobotParts(const unicycleSettings& cfg);
};

std::shared_ptr<UnicyclesWithRods> create_robots(const unicycleSettings& cfg);
std::shared_ptr<Obstacles> create_obs(const YAML::Node &env);

class UnicyclesStateSpace : public ob::CompoundStateSpace
{
public:
    class StateType : public ob::CompoundStateSpace::StateType
    {
    public:
        StateType() = default;
        const Eigen::Vector3f getRobotSt(const size_t idx) const;
        const Eigen::Vector3f getRodSt(const size_t idx) const;
    };

    UnicyclesStateSpace(size_t num_robots, std::vector<double> envmin, std::vector<double> envmax);
    ~UnicyclesStateSpace() override = default;

    size_t getNumRobots() const;

    void setPositionBounds(const ob::RealVectorBounds &bounds);
    const ob::RealVectorBounds &getPositionBounds() const;
    
    void setRotationBounds(const ob::RealVectorBounds &bounds); // alpha bounds

    void setRodBounds(const ob::RealVectorBounds &bounds);
   
    const ob::RealVectorBounds &getRodBounds() const;

    bool satisfiesBounds(const ob::State *state) const override;

    ob::State *allocState() const override;
    void freeState(ob::State *state) const override;

protected:
    size_t num_robots_;
    std::vector<double> env_min_;
    std::vector<double> env_max_;

};