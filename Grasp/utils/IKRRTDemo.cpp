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
#include "GraspPlannerIKParams.hpp"

int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "IK-RRT Demo");
    std::cout << " --- START --- " << std::endl;

    GraspPlannerIKParams params;
    if (!load_GraspPlannerIKParams("/home/nacho/ActiveGrasping/active-grasping/config/scenes/test.json", params)) {
        exit(1);
    }

    GraspPlannerIKui plannerUI(params);

    plannerUI.main();

    return 0;

}
