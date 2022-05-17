
#pragma once

#include <string>
#include <Eigen/Geometry>

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/Workspace/Reachability.h>
#include <VirtualRobot/Grasping/Grasp.h>
#include <VirtualRobot/Grasping/GraspSet.h>
#include <VirtualRobot/CollisionDetection/CDManager.h>

#include <MotionPlanning/Saba.h>
#include <MotionPlanning/CSpace/CSpacePath.h>

#include <GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h>

#include "../include/Grasp/GraspExecutor.hpp"
#include "../include/Grasp/GraspResult.hpp"

#include "GraspPlannerIKParams.hpp"

class GraspPlannerIK : public Grasp::GraspExecutor {
public:
    GraspPlannerIK(const GraspPlannerIKParams& params);


    Grasp::GraspResult executeQueryGrasp(const std::vector<double>& query);

    Grasp::GraspResult executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy);

    Grasp::GraspResult executeGrasp(const Eigen::Matrix4f& targetPose);

    void printInfo();

protected:
    /// INIT

    void loadScene();

    void loadReach();

    /// EEF

    bool plan(Eigen::Matrix4f targetPose);

    void closeEEF();

    void openEEF();

    /**
     * @brief Measure the quality of the current grasp
     * 
     * @return GraspResult
     */
    Grasp::GraspResult graspQuality();

    /// OTHERS

    void reset();

    /// Attributes
    GraspPlannerIKParams params;

    VirtualRobot::RobotPtr robot;
    VirtualRobot::RobotNodeSetPtr rns;
    Eigen::VectorXf startConfig;

    VirtualRobot::EndEffectorPtr eef;
    VirtualRobot::EndEffector::ContactInfoVector contacts;

    VirtualRobot::GraspSetPtr graspSet;
    GraspStudio::GraspQualityMeasureWrenchSpacePtr qualityMeasure;

    VirtualRobot::ManipulationObjectPtr object;
    VirtualRobot::CDManagerPtr cdm;

    /// Common planner params
    bool useCollision;

    /// IK Solver params
    bool useReachability, useOnlyPosition;
    VirtualRobot::ReachabilityPtr reachSpace;

    float ikMaxErrorPos, ikMaxErrorOri; // mm, rads
    float ikJacobianStepSize;
    int ikJacobianMaxLoops, ikMaxLoops = 50;

    /// BiRRT params
    float cspacePathStepSize, cspaceColStepSize;
    int optOptimzeStep = 100;
    Saba::CSpaceSampledPtr cspace;
    Saba::CSpacePathPtr birrtSolution;
    Saba::CSpacePathPtr birrtSolOptimized;
};
