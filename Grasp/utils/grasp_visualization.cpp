#include <iostream>

#include <VirtualRobot/VirtualRobot.h>
#include <Eigen/Geometry>

#include "../include/Grasp/GraspPlannerParams.hpp"
#include "GraspPlannerWindow.h"

int main(int argc, char *argv[]) {

    if (argc !=2) {
        std::cout   << "Error: incorrect number of parameters!!!\n"
                    << "Execution: ./gras_visualization <grasp_params_file_path>\n";
        exit(1);
    }

    std::string params_file = argv[1];

    VirtualRobot::init(argc, argv, "Simox Grasp Planner");

    Grasp::GraspPlannerParams plannerParams;
    if (!Grasp::load_GraspPlannerParams_json(params_file, plannerParams)) {
        std::cout << "Error: parsing grasp planner params file\n";
        exit(1);
    }

    GraspPlannerWindow graspPlanner(plannerParams);

    graspPlanner.main();

    return 0;
}
