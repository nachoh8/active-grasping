#include <iostream>
#include <memory>

#include <bayesopt/parameters.hpp>

#include <grasp/Grasp.hpp>

#include <ActiveGrasping/ActiveGraspingOptParams.h>
#include <ActiveGrasping/ActiveGraspingOpt.h>

void test_gramacy() {
    std::cout << "Test Gramacy\n";

    ActiveGrasping::ActiveGraspingOptParams params;
    params.active_variables.push_back(0);
    params.active_variables.push_back(1);
    params.n_grasp_trials = 1;

    const int opt_dim = params.active_variables.size();
    params.lower_bound = vectord(opt_dim, 0);
    params.upper_bound = vectord(opt_dim, 1);

    params.default_query = vectord(2, 0);

    std::shared_ptr<Grasp::GraspExecutor> executor = std::make_shared<Grasp::TestGramacyExecutor>();
    params.executor = executor;

    // bopt_params bo_params = ActiveGraspingOpt::initBoptParams();
    bopt_params opt_param;
    opt_param       = initialize_parameters_to_default();

    ActiveGrasping::ActiveGraspingOpt opt(params, opt_param);

    vectord best_point(2);

    opt.optimize(best_point);

    for (auto& pt : best_point) {
        std::cout << pt << std::endl;
    }

    std::cout << "END TEST\n";
}


void test_simox() {

    std::cout << "Test Bayes EEF\n";

    /// Set params

    ActiveGrasping::ActiveGraspingOptParams params;
    //params.active_variables.push_back(GRASP_VAR::TRANS_X);
    params.active_variables.push_back(Grasp::GRASP_VAR::TRANS_Y);
    params.active_variables.push_back(Grasp::GRASP_VAR::TRANS_Z);
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
    params.default_query = vectord(Grasp::NUM_GRASP_VARS, 0);

    Eigen::Vector3f obj_position(93, 34, 45);
    Eigen::Vector3f obj_orientation(1.4, 2.84, -3.1);
    
    Grasp::GraspPlannerParams plannerParams(
        "/home/nacho/ActiveGrasping/simox/VirtualRobot/data/robots/iCub/iCub.xml",
        "Left Hand",
        "Grasp Preshape",
        "/home/nacho/ActiveGrasping/simox/VirtualRobot/data/objects/WaterBottleSmall.xml",
        1000.0f, 0.01, true
    );

    plannerParams.obj_pose = true;
    plannerParams.obj_position = obj_position;
    plannerParams.obj_orientation = obj_orientation;

    std::shared_ptr<Grasp::GraspExecutor> executor = std::make_shared<Grasp::GraspPlanner>(plannerParams);
    params.executor = executor;

    /// Optimize

    bopt_params opt_param;
    opt_param       = initialize_parameters_to_default();

    ActiveGrasping::ActiveGraspingOpt opt(params, opt_param);

    vectord best_grasp(opt_dim);

    opt.optimize(best_grasp);

    /// Show result

    std::cout << "Result:\n";
    for (auto& pt : best_grasp) {
        std::cout << pt << std::endl;
    }
    

    std::cout << "END TEST\n";
}


int main(int argc, char *argv[]) {

    // test_gramacy();
    test_simox();

    return 0;
}
