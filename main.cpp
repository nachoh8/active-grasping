#include <iostream>
#include <memory>

#include <bayesopt/parameters.hpp>

#include "include/bayes/ActiveGraspingOptParams.h"
#include "include/bayes/ActiveGraspingOpt.h"

#include "include/utils/GraspVars.hpp"

#include "include/bayes/TestGramacyExecutor.hpp"
#include "include/grasp/GraspPlanner.hpp"
#include "include/grasp/GraspPlannerWindow.h"

void test_gramacy() {
    std::cout << "Test Gramacy\n";

    ActiveGraspingOptParams params;
    params.active_variables[0] = true;
    params.active_variables[1] = true;
    params.object = "Pepino";
    params.n_grasp_trials = 1;
    params.lower_bound = vectord(2, 0);
    params.upper_bound = vectord(2, 1);

    std::shared_ptr<GraspExecutor> executor = std::make_shared<TestGramacyExecutor>();
    params.executor = executor;

    // bopt_params bo_params = ActiveGraspingOpt::initBoptParams();
    bopt_params opt_param;
    opt_param       = initialize_parameters_to_default();

    ActiveGraspingOpt opt(params, opt_param);

    vectord best_point(2);

    opt.optimize(best_point);

    for (auto& pt : best_point) {
        std::cout << pt << std::endl;
    }

    std::cout << "END TEST\n";
}

void test_eef_xy() {
    std::cout << "Test Bayes EEF\n";

    /// Set params

    ActiveGraspingOptParams params;
    //params.active_variables.push_back(GRASP_VAR::TRANS_X);
    params.active_variables.push_back(GRASP_VAR::TRANS_Y);
    params.active_variables.push_back(GRASP_VAR::TRANS_Z);
    /*params.active_variables.push_back(GRASP_VAR::ROT_ROLL);
    params.active_variables.push_back(GRASP_VAR::ROT_PITCH);
    params.active_variables.push_back(GRASP_VAR::ROT_YAW);*/

    const int opt_dim = params.active_variables.size();

    params.object = "WaterBottle";
    params.n_grasp_trials = 1;
    params.lower_bound = vectord(opt_dim, 0); // 0: X, 1: Y, 2: Z, 3: RX, 4: RY:, RZ: 5
    //params.lower_bound[0] = 0;
    params.lower_bound[0] = -110;
    params.lower_bound[1] = -2;
    // params.lower_bound[3] = -3.14;
    // params.lower_bound[4] = -3.14;
    // params.lower_bound[5] = -3.14;
    params.upper_bound = vectord(opt_dim, 1);
    // params.upper_bound[0] = 60;
    params.upper_bound[0] = 20;
    params.upper_bound[1] = 18;
    // params.upper_bound[3] = 3.14;
    // params.upper_bound[4] = 3.14;
    // params.upper_bound[5] = 3.14;
    params.default_query = vectord(NUM_GRASP_VARS, 0);


    Eigen::Vector3f obj_position(93, 34, 45);
    Eigen::Vector3f obj_orientation(1.4, 2.84, -3.1);
    
    GraspPlannerParams plannerParams(
        "/home/nacho/ActiveGrasping/simox/VirtualRobot/data/robots/iCub/iCub.xml",
        "Left Hand",
        "Grasp Preshape",
        "/home/nacho/ActiveGrasping/simox/VirtualRobot/data/objects/WaterBottleSmall.xml",
        1000.0f, 0.01, true
    );

    plannerParams.obj_pose = true;
    plannerParams.obj_position = obj_position;
    plannerParams.obj_orientation = obj_orientation;

    std::shared_ptr<GraspExecutor> executor = std::make_shared<GraspPlanner>(plannerParams);
    params.executor = executor;

    /// Optimize

    bopt_params opt_param;
    opt_param       = initialize_parameters_to_default();

    ActiveGraspingOpt opt(params, opt_param);

    vectord best_grasp(opt_dim);

    opt.optimize(best_grasp);

    /// Show result

    std::cout << "Result:\n";
    for (auto& pt : best_grasp) {
        std::cout << pt << std::endl;
    }

    Eigen::Vector3f eef_position(params.default_query[0], params.default_query[1], params.default_query[2]);
    Eigen::Vector3f eef_orientation(params.default_query[3], params.default_query[4], params.default_query[5]);

    for (int i = 0; i < opt_dim; i++) {
        int idx = params.active_variables[i];
        if (idx < 3) {
            eef_position[idx] = best_grasp[i];
        } else {
            eef_orientation[idx-3] = best_grasp[i];
        }
    }

    plannerParams.eef_pose = true;
    plannerParams.eef_position = eef_position;
    plannerParams.eef_orientation = eef_orientation;

    GraspPlannerWindow graspPlanner(plannerParams);

    graspPlanner.main();
    

    std::cout << "END TEST\n";
}


int main(int argc, char *argv[]) {
    
    VirtualRobot::init(argc, argv, "Simox Grasp Planner");

    test_eef_xy();

    return 0;
}
