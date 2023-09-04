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
    Obstacles(const YAML::Node &env);
    std::vector<fcl::CollisionObjectf *> obstacles;
    fcl::BroadPhaseCollisionManagerf* obsmanager;
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
    size_t numofcables;
    float timelimit;
    float angle_min;
    float angle_max;
    float interpolate;
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
};
