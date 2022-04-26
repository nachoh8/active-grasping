import numpy as np

from bayesoptmodule import BayesOptContinuous

from pygrasp.pygrasp import GraspExecutor, GraspResult

from .active_grasping_params import ActiveGraspingParams
from .datalog import DataLog

class ActiveGrasping(BayesOptContinuous):
    
    def __init__(self, params: ActiveGraspingParams, opt_params: dict, logger: DataLog = None):
        super().__init__(len(params.active_variables))

        self.default_query: np.ndarray = params.default_query
        self.work_dim: int = params.default_query.shape[0]
        self.active_variables: list[int] = params.active_variables
        self.n_grasp_trials: int = params.n_grasp_trials
        self.executor: GraspExecutor = params.executor

        self.params = opt_params
        self.lower_bound = params.lower_bound
        self.upper_bound = params.upper_bound
        
        self.logger: DataLog = logger
        if self.logger:
            self.logger.setup(params, opt_params)

    def evaluateSample(self, x_in) -> float:
        if self.n_dim < self.work_dim:
            query = self.createOptQuery(x_in)
        else:
            query = x_in
        
        qualities = self.applyQueryToHand(query)

        # res = self.evaluateGraspQuality(qualities)
        res = qualities[0]
        
        if self.logger:
            self.logger.add_grasp(list(x_in), res)

        return -res.measure

    ### PRIVATE METHODS

    def createOptQuery(self, query: np.ndarray) -> np.ndarray:
        opt_query = self.default_query.copy()
        for i in range(self.n_dim):
            idx = self.active_variables[i]
            opt_query[idx] = query[i]
        
        return opt_query

    def applyQueryToHand(self, query: np.ndarray) -> "list[GraspResult]":
        res: list[GraspResult] = []
        for _ in range(self.n_grasp_trials):
            r = self.executor.executeQueryGrasp(query)

            res.append(r)
        
        return res

    def evaluateGraspQuality(self, qualities: "list[GraspResult]") -> float:
        n = len(qualities)

        if n == 0: return 0.0
        
        measure = 0.0
        volume = 0.0
        force_closure = 0.0

        for res in qualities:
            measure += res.measure
            volume += res.volume
            force_closure += float(res.force_closure)
        
        measure         /= n
        volume          /= n
        force_closure   /= n

        return measure, volume, force_closure
