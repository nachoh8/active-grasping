#include "../include/Grasp/GraspPlannerSParams.hpp"

#include <iostream>

#include <boost/property_tree/json_parser.hpp>

namespace pt = boost::property_tree;

bool parse_eigen(pt::ptree& json_vec, Eigen::Vector3f& vec) {
    std::vector<float> values;
    for (pt::ptree::value_type &v : json_vec)
    {
        float num = v.second.get_value<float>();
        values.push_back(num);
    }

    if (values.size() == 3) {
        vec = Eigen::Vector3f(values[0], values[1], values[2]);

        return true;
    }

    return false;
}

bool parse_pose(pt::ptree& root, Eigen::Vector3f& pos, Eigen::Vector3f& rot) {
    try {
        bool c_pos = parse_eigen(root.get_child("position"), pos);
        bool c_rot = parse_eigen(root.get_child("orientation"), rot);

        return c_pos && c_rot;
    } catch(std::exception & e) {

        return false;
    }
}

namespace Grasp {

bool load_GraspPlannerSParams_json(const std::string& json, GraspPlannerParams& params) {
    pt::ptree root;
    std::string robot_json, obj_json;
    try {
        pt::read_json(json, root);

        params.timeout = root.get<float>("timeout");
        params.min_quality = root.get<float>("min_quality");
        params.force_closure = root.get<bool>("force_closure");

        robot_json = root.get<std::string>("robot");
        obj_json = root.get<std::string>("object");
    } catch(std::exception & e) {
        std::cout << "Error loading GraspPlannerParams from file: " << e.what() << std::endl;

        return false;
    }

    try {
        pt::read_json(robot_json, root);

        params.robot_file = root.get<std::string>("file");
        params.eef_name = root.get<std::string>("eef_name");
        params.preshape = root.get<std::string>("preshape");

        params.has_eef_pose = parse_pose(root, params.eef_position, params.eef_orientation);
    } catch(std::exception & e) {
        std::cout << "Error loading GraspPlannerParams from file: " << e.what() << std::endl;

        return false;
    }

    try {
        pt::read_json(obj_json, root);

        params.object_file = root.get<std::string>("file");

        params.has_obj_pose = parse_pose(root, params.obj_position, params.obj_orientation);
    } catch(std::exception & e) {
        std::cout << "Error loading GraspPlannerParams from file: " << e.what() << std::endl;

        return false;
    }

    return true;
}

/*bool load_GraspPlannerParams_json(const std::string& json, GraspPlannerParams& params) {
    pt::ptree root;
    try {
        pt::read_json(json, root);

        params.robot_file = root.get<std::string>("robot_file");
        params.eef_name = root.get<std::string>("eef_name");
        params.preshape = root.get<std::string>("preshape");
        params.object_file = root.get<std::string>("object_file");

        params.timeout = root.get<float>("timeout");
        params.min_quality = root.get<float>("min_quality");
        params.force_closure = root.get<bool>("force_closure");

        params.has_eef_pose = parse_pose(root, "eef_position", "eef_orientation", params.eef_position, params.eef_orientation);
        params.has_obj_pose = parse_pose(root, "obj_position", "obj_orientation", params.obj_position, params.obj_orientation);

        return true;
    } catch(std::exception & e) {
        std::cout << "Error loading GraspPlannerParams from file: " << e.what() << std::endl;

        return false;
    }
}*/

}

