#include <iostream>

#include <VirtualRobot/VirtualRobot.h>
#include <Eigen/Geometry>

#include <grasp/GraspPlannerWindow.h>
#include <grasp/GraspPlannerParams.hpp>

int main(int argc, char *argv[]) {
    
    std::cout << "Test ActiveGrasping window\n";

    VirtualRobot::init(argc, argv, "Simox Grasp Planner");

    Eigen::Vector3f obj_position(93, 34, 45);
    Eigen::Vector3f obj_orientation(1.4, 2.84, -3.1);

    Eigen::Vector3f eef_position(49.59, -109.12, 0);
    Eigen::Vector3f eef_orientation(0, 0, 0);
    
    Grasp::GraspPlannerParams plannerParams(
        "/home/nacho/ActiveGrasping/simox/VirtualRobot/data/robots/iCub/iCub.xml",
        "Left Hand",
        "Grasp Preshape",
        "/home/nacho/ActiveGrasping/simox/VirtualRobot/data/objects/WaterBottleSmall.xml",
        1000.0f, 0.01, true
    );

    plannerParams.eef_pose = false;
    plannerParams.eef_position = eef_position;
    plannerParams.eef_orientation = eef_orientation;

    plannerParams.obj_pose = true;
    plannerParams.obj_position = obj_position;
    plannerParams.obj_orientation = obj_orientation;

    Grasp::GraspPlannerWindow graspPlanner(plannerParams);

    graspPlanner.main();

    std::cout << "END TEST\n";

    return 0;
}
