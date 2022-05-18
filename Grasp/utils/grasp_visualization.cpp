#include <iostream>
#include <vector>

#include <boost/property_tree/json_parser.hpp>

#include <VirtualRobot/VirtualRobot.h>
#include <Eigen/Geometry>

#include "../include/Grasp/GraspPlannerParams.hpp"
#include "../include/Grasp/GraspVars.hpp"
#include "GraspPlannerWindow.h"

namespace pt = boost::property_tree;

inline int str_to_var(const std::string& var) {
    if (var == "x") {
        return Grasp::GRASP_VAR::TRANS_X;
    } else if (var == "y") {
        return Grasp::GRASP_VAR::TRANS_Y;
    } else if ( var == "z") {
        return Grasp::GRASP_VAR::TRANS_Z;
    } else if ( var == "rx") {
        return Grasp::GRASP_VAR::ROT_ROLL;
    } else if ( var == "ry") {
        return Grasp::GRASP_VAR::ROT_PITCH;
    } else if ( var == "rz") {
        return Grasp::GRASP_VAR::ROT_YAW;
    } else {
        return -1;
    }
}

bool load_grasps_from_res(const std::string& file, std::vector<Grasp::GraspData>& grasps) {
    pt::ptree root_grasps;
    std::vector<int> active_variables;
    Eigen::Vector3f default_pos, default_ori;
    try {
        pt::ptree root;
        pt::read_json(file, root);
        pt::ptree root_gopt = root.get_child("gopt_params");
        
        // parse active variables
        pt::ptree root_vars = root_gopt.get_child("active_variables");
        for (pt::ptree::value_type &v : root_vars)
        {
            std::string str_var = v.second.get_value<std::string>();
            int var = str_to_var(str_var);
            if (var < Grasp::GRASP_VAR::TRANS_X || var > Grasp::GRASP_VAR::ROT_YAW) {
                std::cout << "Error: active variable " << str_var << " not valid!\n";
                return false;
            }
            active_variables.push_back(var);
        }

        // parse default query
        pt::ptree root_q = root_gopt.get_child("default_query");
        std::vector<float> default_pose;
        for (pt::ptree::value_type &v : root_q)
        {
            float num = v.second.get_value<float>();
            default_pose.push_back(num);
        }

        if (default_pose.size() != 6) {
            std::cout << "Error: default query size is different from 6!\n";
            return false;
        }

        default_pos = Eigen::Vector3f(default_pose[GRASP_VAR::TRANS_X], default_pose[GRASP_VAR::TRANS_Y], default_pose[GRASP_VAR::TRANS_Z]);
        default_ori = Eigen::Vector3f(default_pose[GRASP_VAR::ROT_ROLL], default_pose[GRASP_VAR::ROT_PITCH], default_pose[GRASP_VAR::ROT_YAW]);

        // get grasps root
        root_grasps = root.get_child("grasps");
    } catch(std::exception & e) {
        std::cout << "Error loading Grasps from file: " << e.what() << std::endl;

        return false;
    }

    // parse grasps
    try {
        for (auto &grasp_json : root_grasps)
        {
            pt::ptree grasp_obj = grasp_json.second;
            pt::ptree root_query = grasp_obj.get_child("query");
            
            std::vector<float> q;
            for (pt::ptree::value_type &v : root_query)
            {
                float val = v.second.get_value<float>();
                q.push_back(val);
            }

            Eigen::Vector3f pos = default_pos, ori = default_ori;
            for (int i = 0; i < active_variables.size(); i++) {
                int var = active_variables[i];
                if (var < Grasp::GRASP_VAR::ROT_ROLL) {
                    pos[var] = q[i];
                } else {
                    ori[var] = q[i];
                }
            }

            pt::ptree root_res = grasp_obj.get_child("res");
            Grasp::GraspData grasp;
            grasp.result.measure = root_res.get<float>("measure");
            grasp.result.volume = root_res.get<float>("volume");
            grasp.result.force_closure = root_res.get<bool>("force_closure");
            grasp.pos = pos;
            grasp.ori = ori;

            grasps.push_back(grasp);
        }

    } catch (std::exception & e) {
        std::cout << "Error parsing grasps: " << e.what() << std::endl;

        return false;
    }

    return true;
}

int main(int argc, char *argv[]) {

    if (argc < 2 || argc > 3) {
        std::cout   << "Error: incorrect number of parameters!!!\n"
                    << "Execution: ./grasp_visualization <grasp_params_file_path> [<res_file>]\n";
        exit(1);
    }

    std::string params_file = argv[1];

    VirtualRobot::init(argc, argv, "Simox Grasp Planner");

    Grasp::GraspPlannerParams plannerParams;
    if (!Grasp::load_GraspPlannerParams_json(params_file, plannerParams)) {
        std::cout << "Error: parsing grasp planner params file\n";
        exit(1);
    }

    GraspPlannerWindowParams params;
    params.planner_params = plannerParams;

    if (argc == 3) {
        std::string res_file = argv[2];
        if (!load_grasps_from_res(res_file, params.grasps)) {
            std::cout << "Error: parsing grasps from res file\n";
            exit(1);
        }
    }

    GraspPlannerWindow graspPlanner(params);

    graspPlanner.main();

    return 0;
}
