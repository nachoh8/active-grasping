#pragma once

#include <string>
#include <memory>
#include <vector>

#include <specialtypes.hpp>

#include "GraspExecutor.hpp"

typedef std::shared_ptr<GraspExecutor> GraspExecutorPtr;

struct ActiveGraspingOptParams
{
    std::string     object;
    unsigned int    grasp;
    unsigned int    position;

    unsigned int    n_grasp_trials;
    double          trial_stddev;
    double          grasp_threshold;

    vectord default_query;
    vectord lower_bound, upper_bound;
    std::vector<int> active_variables;

    GraspExecutorPtr executor;
};
