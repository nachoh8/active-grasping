import numpy as np

from bayesoptmodule import BayesOptContinuous

from pygrasp.pygrasp import GraspResult

from .grasp_models import ExecutorModel, GraspPlannerIKExecutor
from .active_grasping import ActiveGrasping
from .datalog import DataLog

class BayesOptExecutor(ActiveGrasping, BayesOptContinuous):
    
    def __init__(self, params: dict, executor: ExecutorModel, logger: DataLog = None):
        n_grasp_trials = int(params['grasp_trials'])
        active_variables = params['active_variables']
        default_query = params['default_query']
        lower_bound = np.array(params['lower_bound'], dtype=np.float64)
        upper_bound = np.array(params['upper_bound'], dtype=np.float64)
        
        ActiveGrasping.__init__(self, executor, active_variables, default_query, ["outcome", "volume", "force_closure"], n_trials=n_grasp_trials, logger=logger)
        BayesOptContinuous.__init__(self, len(active_variables))

        self.params = params['bopt_params']
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        
        if self.logger:
            optimizer_log = {"lower_bound": list(self.lower_bound), "upper_bound": list(self.upper_bound), "bopt_params": self.params}
            self.logger.log_optimizer("bayesopt", optimizer_log)

    def run(self) -> None:
        print("------------------------")
        print("BAYESOPT")
        print("Active variables: " + str(self.active_variables))
        print("Default query: " + str(self.default_query))
        print("------------------------")

        quality, x_out, _ = self.optimize()     

        r = {"query": dict(zip(self.active_variables, list(x_out))), "metrics": [{"name":"outcome", "value": -quality}]}
        self.best_results = [r]

        print("------------------------")
        print("Best:")
        print("\tPoint:", x_out)
        print("\tOutcome:", -quality)
        
    def evaluateSample(self, x_in) -> float:
        query = dict(zip(self.active_variables, list(x_in)))
        res: GraspResult = self.executeQuery(query)
        
        return -res.measure

