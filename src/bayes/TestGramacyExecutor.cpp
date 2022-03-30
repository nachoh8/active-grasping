#include "bayes/TestGramacyExecutor.hpp"

#include <cmath>

inline double sqr(double x) {return x*x;};

GraspResult TestGramacyExecutor::executeQueryGrasp(const vectord& query) {
    vectord x = query;
    
    if (x.size() != 2) return GraspResult();

    x(0) = (x(0) * 8) - 2;
    x(1) = (x(1) * 8) - 2;

    double r = (x(0) * std::exp(-sqr(x(0))-sqr(x(1))));

    return GraspResult(r, 0.0f, false);
}
