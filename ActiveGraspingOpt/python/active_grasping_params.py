import numpy as np

import json

from pygrasp.pygrasp import GraspExecutor
from pygrasp.gutils import str_to_var

class ActiveGraspingParams:
    def __init__(self, default_query: np.ndarray, active_variables: "list[int]",
                lower_bound: np.ndarray, upper_bound: np.ndarray,
                n_grasp_trials: int, executor: GraspExecutor = None):

        self.n_grasp_trials = n_grasp_trials
        self.active_variables: list[int] = active_variables
        self.lower_bound: np.ndarray = lower_bound
        self.upper_bound: np.ndarray = upper_bound
        self.default_query: np.ndarray = default_query
        self.executor: GraspExecutor = executor

        if n_grasp_trials < 1:
            raise Exception("ActiveGraspingParams: n_grasp_trials must be equal or greather than 1")
        
        n = len(active_variables)
        if n == 0:
            raise Exception("ActiveGraspingParams: active_variables must not be empty")
        
        if n != lower_bound.shape[0] or n != upper_bound.shape[0]:
            raise Exception("ActiveGraspingParams: bounds size not match")


def load_ActiveGraspingParams(json_file: str, executor: GraspExecutor) -> ActiveGraspingParams:
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    n_gras_trials = int(data['grasp_trials'])
    active_variables = [str_to_var(var) for var in data['active_variables']]
    lower_bound = np.array(data['lower_bound'], dtype=np.float64)
    upper_bound = np.array(data['upper_bound'], dtype=np.float64)
    default_query = np.array(data['default_query'], dtype=np.float64)

    return ActiveGraspingParams(default_query, active_variables, lower_bound, upper_bound, n_gras_trials, executor)
