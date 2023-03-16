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
    size_t getObsNum();
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
};

class nCablesPayload
{
public:
    std::shared_ptr<ob::SpaceInformation> si;
    
    nCablesPayload(const plannerSettings& cfg);

    void addRobotParts(const plannerSettings& cfg);
    fcl::CollisionObjectf* payloadObj;
    std::vector<fcl::CollisionObjectf *> cablesObj;
    std::vector<fcl::CollisionObjectf *> uavObj;

    fcl::Transform3f getPayloadTransform(const ompl::base::State *state);
    fcl::Transform3f getCableTransform(const ompl::base::State *state, const size_t cableNum, Eigen::Vector3f& attachmentPoint,const double length);
    fcl::Transform3f getUAVTransform(const ompl::base::State *state, const size_t cableNum, Eigen::Vector3f& attachmentPoint,const double length);
    
    void setPayloadTransformation(const ob::State *state);
    void setCableTransformation(const ob::State *state, const size_t cableNum, Eigen::Vector3f& attachmentPoint,const double length);
    void setUAVTransformation(const ob::State *state,   const size_t uavNum, Eigen::Vector3f& attachmentPoint,const double length);

    fcl::BroadPhaseCollisionManagerf* sysparts;
    // fcl::BroadPhaseCollisionManagerf* cableuavpart;
    // fcl::BroadPhaseCollisionManagerf* payloadpart;
    void setSysParts();
};

std::shared_ptr<nCablesPayload> create_sys(const plannerSettings& cfg);
std::shared_ptr<Obstacles> create_obs(const YAML::Node &env);

class StateSpace : public ob::CompoundStateSpace
{
public:
class StateType : public ob::CompoundStateSpace::StateType
{
public:
    StateType() = default;

    const Eigen::Vector3f getPayloadPos() const
    {
        auto pos =  as<ob::RealVectorStateSpace::StateType>(0)->values;
        Eigen::Vector3f payloadPos;
        for(size_t i = 0; i < 3; ++i) 
        {
            payloadPos(i) = pos[i];
        }
        return payloadPos;
    }

    void setPayloadPos(const Eigen::Vector3f& payloadPos) const
    {
        auto pos =  as<ob::RealVectorStateSpace::StateType>(0)->values;
        pos[0] = payloadPos(0);
        pos[1] = payloadPos(1);
        pos[2] = payloadPos(2);
    }
    const Eigen::Quaternionf getPayloadquat() const 
    {
        auto rot = as<ob::SO3StateSpace::StateType>(1);
        const Eigen::Quaternionf payload_quat(rot->w, rot->x, rot->y, rot->z);
        return payload_quat;
    }
    void setPayloadquat(const Eigen::Quaternionf& payload_quat)
    {
        auto rot = as<ob::SO3StateSpace::StateType>(1);
        rot->w = payload_quat.w();
        rot->x = payload_quat.x();
        rot->y = payload_quat.y();
        rot->z = payload_quat.z();
    } 
    Eigen::Vector3f getCablePos(const size_t cableNum, Eigen::Vector3f& attachmentPoint,const double& length) const
    {    
        Eigen::Vector3f unitvec = getunitvec(cableNum);
        Eigen::Vector3f attPointInFixedFrame = getAttPointInFixedFrame(attachmentPoint);
        return attPointInFixedFrame + length*unitvec;
    }

    Eigen::Quaternionf getCableQuat(size_t cableNum) const 
    {
        Eigen::Vector3f unitvec = getunitvec(cableNum);
        Eigen::Vector3f basevec(0,0,1);
        Eigen::Quaternionf quat = Eigen::Quaternionf::FromTwoVectors(basevec, unitvec);
        return quat;
    }

    Eigen::Vector3f getuavPos(const size_t& cableNum, Eigen::Vector3f& attachmentPoint,const double& length) const
    {    
        Eigen::Vector3f unitvec = getunitvec(cableNum);
        Eigen::Vector3f attPointInFixedFrame = getAttPointInFixedFrame(attachmentPoint);
        return attPointInFixedFrame + (length+0.15)*unitvec;
    }

    Eigen::Vector3f getunitvec(size_t cablenum) const
    {    
        auto az = as<ob::RealVectorStateSpace::StateType>(2)->values[0+2*cablenum];
        auto el = as<ob::RealVectorStateSpace::StateType>(2)->values[1+2*cablenum];
        
        Eigen::Vector3f unitvec(std::cos(az)*std::sin(el), std::sin(az)*std::cos(el), std::sin(el));
        return unitvec;
    }
protected:

    Eigen::Vector3f getAttPointInFixedFrame(Eigen::Vector3f& attachmentPoint) const
    {    
        Eigen::Vector3f payloadPos = getPayloadPos();
        Eigen::Quaternionf payload_quat = getPayloadquat();

        Eigen::Vector3f attPointInFixedFrame = payloadPos + payload_quat.normalized().toRotationMatrix() * attachmentPoint;
        return attPointInFixedFrame;
    }

};

StateSpace(size_t numCables)
{
    setName("Quadrotor" + getName());
    type_ = ob::STATE_SPACE_TYPE_COUNT + 0;
    addSubspace(std::make_shared<ob::RealVectorStateSpace>(3), 1.0);      // position
    addSubspace(std::make_shared<ob::SO3StateSpace>(), 1.0);              // orientation
    addSubspace(std::make_shared<ob::RealVectorStateSpace>(2*numCables), 1.0); // cable az and el
    lock();
}

~StateSpace() override = default;

size_t getNumCables() const
{
    return as<ob::RealVectorStateSpace>(2)->getDimension() / 2;
}

void setPositionBounds(const ob::RealVectorBounds &bounds)
{
    as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
}

void setCableBounds(const ob::RealVectorBounds &bounds) 
{
    as<ob::RealVectorStateSpace>(2)->setBounds(bounds);
}

const ob::RealVectorBounds &getPositionBounds() const
{
    return as<ob::RealVectorStateSpace>(0)->getBounds();
}

ob::State *allocState() const override
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

void freeState(ob::State *state) const override
{
    CompoundStateSpace::freeState(state);
}

void registerProjections()
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
};
