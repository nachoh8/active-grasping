#pragma once

#include <string>

#include <Eigen/Geometry>

namespace Grasp {

struct GraspPlannerParams {
    std::string robot_file;
    std::string eef_name;
    std::string preshape;
    std::string object_file;

    bool eef_pose = false;
    Eigen::Vector3f eef_position, eef_orientation;

    bool obj_pose = false;
    Eigen::Vector3f obj_position, obj_orientation;

    float timeout;
    float min_quality;
    bool force_closure;

    GraspPlannerParams() {}

    GraspPlannerParams(
        const std::string& _robot_file,
        const std::string& _eef_name,
        const std::string& _preshape,
        const std::string& _object_file,
        const float _timeout, const float _min_quality, const bool _force_closure) {

        robot_file = _robot_file;
        eef_name = _eef_name;
        preshape = _preshape;
        object_file = _object_file;

        timeout = _timeout;
        min_quality = _min_quality;
        force_closure = _force_closure;
    }
};

}
