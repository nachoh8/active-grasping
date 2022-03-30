#pragma once

#include <specialtypes.hpp>

#include "GraspExecutor.hpp"


class TestGramacyExecutor : public GraspExecutor {
public:
    
    GraspResult executeQueryGrasp(const vectord& query);
};
