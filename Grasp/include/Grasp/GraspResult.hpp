#pragma once

namespace Grasp {
struct GraspResult
{
    double measure;
    double volume;
    bool force_closure;

    double time; // ms
    double pos_error; // mm
    double ori_error; // degrees
    float rho; // Computed intersection rho

    GraspResult()
    {
        measure       = 0;
        volume        = 0;
        force_closure = false;

        time = -1;
        pos_error = -1;
        ori_error = -1;

        rho = 0;
    }

    GraspResult(const double _measure, const double _volume, bool _force_closure, float _rho)
    {
        measure       = _measure;
        volume        = _volume;
        force_closure = _force_closure;

        time = -1;
        pos_error = -1;
        ori_error = -1;

        rho = _rho;
    }

    GraspResult(const double _measure, const double _volume, bool _force_closure)
    {
        measure       = _measure;
        volume        = _volume;
        force_closure = _force_closure;

        time = -1;
        pos_error = -1;
        ori_error = -1;
    }

    GraspResult(const double _measure, const double _volume, bool _force_closure, double _time, double _pos_error, double _ori_error, float _rho)
    {
        measure       = _measure;
        volume        = _volume;
        force_closure = _force_closure;

        time = _time;
        pos_error = _pos_error;
        ori_error = _ori_error;

        rho = _rho;
    }
};
}
