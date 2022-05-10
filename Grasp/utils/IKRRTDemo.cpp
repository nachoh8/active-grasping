#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/RuntimeEnvironment.h>

#include <string>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "GraspPlannerIK.hpp"

int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "IK-RRT Demo");
    std::cout << " --- START --- " << std::endl;

    // ICUB
    std::string filenameScene = "/home/nacho/ActiveGrasping/MySimox/VirtualRobot/data/scenes/IKRRT_scene_iCub.xml";
    std::string filenameReach = "/home/nacho/ActiveGrasping/MySimox/VirtualRobot/data/reachability/iCub_HipLeftArm.bin";
    std::string kinChain = "Hip Left Arm";
    std::string eef = "Left Hand";
    std::string colModel = "Left HandArm ColModel";
    std::string colModelRob = "BodyHeadLegsColModel";

    std::cout << "Using scene at " << filenameScene << std::endl;
    std::cout << "Using reachability at " << filenameReach << std::endl;
    std::cout << "Using end effector " << eef << std::endl;
    std::cout << "Using col model (kin chain) " << colModel << std::endl;
    std::cout << "Using col model (static robot)" << colModelRob << std::endl;

    GraspPlannerIK planner(filenameScene, filenameReach, kinChain, eef, colModel, colModelRob);

    return 0;

}
