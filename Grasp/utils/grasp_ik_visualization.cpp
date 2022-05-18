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

#include "../include/Grasp/GraspPlannerIKParams.hpp"
#include "GraspPlannerIKWindow.h"

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cout   << "Error: incorrect number of parameters!!!\n"
                    << "Execution: ./grasp_ik_visualization <params_file>\n";
        exit(1);
    }

    std::string params_file = argv[1];

    VirtualRobot::init(argc, argv, "GraspPlanner IK-BiRRT Demo");
    std::cout << " --- START --- " << std::endl;

    Grasp::GraspPlannerIKParams params;
    if (!Grasp::load_GraspPlannerIKParams(params_file, params)) {
        exit(1);
    }

    GraspPlannerIKWindow plannerUI(params);

    plannerUI.main();

    return 0;

}
