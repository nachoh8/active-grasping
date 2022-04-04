#include "ActiveGrasping/ActiveGraspingOpt.h"

#include <algorithm>
// #include <chrono>

#include <grasp/GraspResult.hpp>

namespace ActiveGrasping {

ActiveGraspingOpt::ActiveGraspingOpt(const ActiveGraspingOptParams& _params, const bopt_params& bo_params)
: params(_params), work_dim(_params.default_query.size()), ymin(0.0f), ymax(1.0f),
    bayesopt::ContinuousModel(_params.active_variables.size(), bo_params)
{
    this->setBoundingBox(params.lower_bound, params.upper_bound);
}

double ActiveGraspingOpt::evaluateSample(const vectord& query) {
    
    vectord opt_query = mDims < work_dim ? createOptQuery(query) : query;

    std::vector<Grasp::GraspResult> qualities = applyQueryToHand(opt_query);

    double res = evaluateGraspQuality(qualities);

    return std::clamp(res, ymin, ymax);
}

vectord ActiveGraspingOpt::createOptQuery(const vectord& query) {
    vectord opt_query = params.default_query;

    for (int i = 0; i < mDims; i++) {
        int idx = params.active_variables[i];
        opt_query[idx] = query[i];
    }

    return opt_query;
}

std::vector<Grasp::GraspResult> ActiveGraspingOpt::applyQueryToHand(const vectord& query) {
    std::vector<Grasp::GraspResult> results;
    
    // unsigned rand_seed = std::chrono::system_clock::now().time_since_epoch().count();
    // std::default_random_engine generator(rand_seed);

    std::vector q = {query[0], query[1]};
    for (int trial = 0; trial < params.n_grasp_trials; trial++) {
        Grasp::GraspResult res = params.executor->executeQueryGrasp(q);

        results.push_back(res);
    }

    return results;
}

double ActiveGraspingOpt::evaluateGraspQuality(const std::vector<Grasp::GraspResult>& qualities) {
    const int n = qualities.size();

    if (n == 0) return 0.0;

    double measure = 0.0, volume = 0.0, force_closure = 0.0;

    for (auto& gRes: qualities) {
        measure += gRes.measure;
        volume += gRes.volume;
        force_closure += gRes.force_closure;
    }

    measure         /= n;
    volume          /= n;
    force_closure   /= n;

    return measure;
}

}
