#pragma once

#include <specialtypes.hpp>

#include "utils/GraspResult.hpp"

class GraspExecutor {
public:
    /**
     * @brief Execute grasp from bayesopt query and computes its quality
     * 
     * @param query EEF position (x,y,z,r,p,y)
     * @return Grasp quality 
     */
    virtual GraspResult executeQueryGrasp(const vectord& query) = 0;
};
