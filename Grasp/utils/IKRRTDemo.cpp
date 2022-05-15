#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/RuntimeEnvironment.h>

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/Qt/SoQt.h>

#include <string>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "GraspPlannerIKui.h"

int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "IK-RRT Demo");
    std::cout << " --- START --- " << std::endl;

    // ICUB
    std::string filenameScene = "/home/nacho/ActiveGrasping/MySimox/VirtualRobot/data/scenes/IKRRT_scene_iCub.xml";
    std::string filenameReach = "/home/nacho/ActiveGrasping/MySimox/VirtualRobot/data/reachability/iCub_HipLeftArm.bin";
    std::string kinChain = "Left Arm";
    std::string kinChainHip = "Hip Left Arm";
    std::string eef = "Left Hand";
    std::string colModel = "Left HandArm ColModel";
    std::string colModelRob = "BodyHeadLegsColModel";

    std::cout << "Using scene at " << filenameScene << std::endl;
    std::cout << "Using reachability at " << filenameReach << std::endl;
    std::cout << "Using end effector " << eef << std::endl;
    std::cout << "Using col model (kin chain) " << colModel << std::endl;
    std::cout << "Using col model (static robot)" << colModelRob << std::endl;

    GraspPlannerIKui plannerUI(filenameScene, filenameReach, kinChainHip, eef, colModel, colModelRob);

    plannerUI.main();

    return 0;

}
