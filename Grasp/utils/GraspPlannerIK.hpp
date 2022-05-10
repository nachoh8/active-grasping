#pragma once

#include <string>
#include <Eigen/Geometry>

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Obstacle.h>
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/Grasping/Grasp.h"

#include <MotionPlanning/Saba.h>
#include <MotionPlanning/CSpace/CSpacePath.h>

#include <GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h>

#include "../include/Grasp/GraspExecutor.hpp"
#include "../include/Grasp/GraspResult.hpp"

class GraspPlannerIK : public Grasp::GraspExecutor {
public:
    GraspPlannerIK(const std::string& sceneFile, const std::string& reachFile, const std::string& rns,
                const std::string& eef, const std::string& colModel, const std::string& colModelRob);


    Grasp::GraspResult executeQueryGrasp(const std::vector<double>& query);

    Grasp::GraspResult executeGrasp(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy, bool save_grasp=true);

protected:
    /// INIT

    void loadScene(const std::string& sceneFile);

    void loadReach(const std::string& reachFile);

    /// EEF

    bool plan(VirtualRobot::GraspPtr targetGrasp);

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

    VirtualRobot::RobotPtr robot;
    Saba::CSpaceSampledPtr cspace;
    Eigen::VectorXf startConfig;

    VirtualRobot::EndEffectorPtr eef;
    VirtualRobot::EndEffector::ContactInfoVector contacts;

    std::vector< VirtualRobot::ObstaclePtr > obstacles;
    VirtualRobot::ManipulationObjectPtr object;
    VirtualRobot::ReachabilityPtr reachSpace;

    // VirtualRobot::GraspSetPtr graspSet;
    VirtualRobot::RobotNodeSetPtr rns;

    GraspStudio::GraspQualityMeasureWrenchSpacePtr qualityMeasure;

    std::string eefName;
    std::string rnsName;
    std::string colModelName;
    std::string colModelNameRob;
};
