#pragma once

namespace Grasp {
struct GraspResult
{
    double measure;
    double volume;
    bool force_closure;

    GraspResult()
    {
        measure       = 0;
        volume        = 0;
        force_closure = false;
    }

    GraspResult(const double _measure, const double _volume, bool _force_closure)
    {
        measure       = _measure;
        volume        = _volume;
        force_closure = _force_closure;
    }
};
}
