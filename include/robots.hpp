#pragma once

// OMPL headers
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <fcl/fcl.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
namespace ob = ompl::base;


class Obstacles
{
public:
    Obstacles(const YAML::Node &env);
    // void addObstacles();
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
    std::vector<double> cableslength;
    std::vector<double> desiredCableStates;
    std::string plannerType;
    std::string payloadShape;
    size_t numofcables;
    float timelimit;
};

class nCablesPayload
{
public:
    nCablesPayload(const plannerSettings& cfg);
    void addRobotParts(const plannerSettings& cfg);
    std::shared_ptr<ob::SpaceInformation> si;
    std::vector<fcl::CollisionObjectf *> systemParts;
    fcl::BroadPhaseCollisionManagerf* sysparts;
};

class StateSpace : public ob::CompoundStateSpace
{
public:
    class StateType : public ob::CompoundStateSpace::StateType
    {
    public: 
        StateType() = default;
        double getX() const
        {
            return as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        }

        double getY() const
        {
            return as<ob::RealVectorStateSpace::StateType>(0)->values[1];
        }

        double getZ() const
        {
            return as<ob::RealVectorStateSpace::StateType>(0)->values[2];
        }

        const Eigen::Quaternionf getPayloadquat() const 
        {
            auto rot = as<ob::SO3StateSpace::StateType>(1);
            const Eigen::Quaternionf payload_quat(rot->w, rot->x, rot->y, rot->z);
            return payload_quat;
        }
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
        Eigen::Vector3f getCablePos(const size_t& cablenum, Eigen::Vector3f& attachmentPoint,const double& length) const
        {    
            Eigen::Vector3f payloadPos = getPayloadPos();
            Eigen::Quaternionf payload_quat = getPayloadquat();
            auto az = as<ob::RealVectorStateSpace::StateType>(2)->values[2*cablenum-2];
            auto el = as<ob::RealVectorStateSpace::StateType>(2)->values[2*cablenum-1];
          
            Eigen::Vector3f unitvec(std::cos(az)*std::sin(el), std::sin(az)*std::cos(el), std::sin(el));
            Eigen::Vector3f attPointInFixedFrame = payloadPos + payload_quat.normalized().toRotationMatrix() * attachmentPoint;
            return attPointInFixedFrame + length*unitvec;
        }

        Eigen::Vector3f getuavPos(const size_t& cablenum, Eigen::Vector3f& attachmentPoint,const double& length) const
        {    
            Eigen::Vector3f payloadPos = getPayloadPos();
            Eigen::Quaternionf payload_quat = getPayloadquat();
            auto az = as<ob::RealVectorStateSpace::StateType>(2)->values[2*cablenum-2];
            auto el = as<ob::RealVectorStateSpace::StateType>(2)->values[2*cablenum-1];
          
            Eigen::Vector3f unitvec(std::cos(az)*std::sin(el), std::sin(az)*std::cos(el), std::sin(el));
            Eigen::Vector3f attPointInFixedFrame = payloadPos + payload_quat.normalized().toRotationMatrix() * attachmentPoint;
            return attPointInFixedFrame + (length+0.15)*unitvec;
        }
    };
};