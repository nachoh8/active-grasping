import numpy as np

import json

from pygrasp import *
from bayesoptmodule import BayesOptContinuous

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
            raise Exception("ActiveGraspingParams: n_grasp_trials must be greather than 1")
        
        n = len(active_variables)
        if n == 0:
            raise Exception("ActiveGraspingParams: active_variables must not be empty")
        
        if n != lower_bound.shape[0] or n != upper_bound.shape[0]:
            raise Exception("ActiveGraspingParams: bounds size not match")

def str_to_var(var: str) -> int:
        if var == "x":
            return TRANS_X
        elif var == "y":
            return TRANS_Y
        elif var == "z":
            return TRANS_Z
        elif var == "rx":
            return ROT_ROLL
        elif var == "ry":
            return ROT_PITCH
        elif var == "rz":
            return ROT_YAW

def var_to_str(var: int) -> str:
        if var == TRANS_X:
            return "x"
        elif var == TRANS_Y:
            return "y"
        elif var == TRANS_Z:
            return "z"
        elif var == ROT_ROLL:
            return "rx"
        elif var == ROT_PITCH:
            return "ry"
        elif var == ROT_YAW:
            return "rz"
        else:
            return var

def load_ActiveGraspingParams(json_file: str, executor: GraspExecutor) -> ActiveGraspingParams:
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    n_gras_trials = int(data['grasp_trials'])
    active_variables = [str_to_var(var) for var in data['active_variables']]
    lower_bound = np.array(data['lower_bound'], dtype=np.float64)
    upper_bound = np.array(data['upper_bound'], dtype=np.float64)
    default_query = np.array(data['default_query'], dtype=np.float64)

    return ActiveGraspingParams(default_query, active_variables, lower_bound, upper_bound, n_gras_trials, executor)


class DataLog(object):
    def __init__(self, log_file: str = "") -> None:
        self.gopt_params: ActiveGraspingParams = None
        self.bopt_params: dict = {}
        self.n_init_samples = 0
        self.n_samples: int = 0
        self.grasps = []
        self.log_file: str = log_file

    def setup(self, gopt_params: ActiveGraspingParams, bopt_params: dict):
        self.bopt_params = bopt_params
        if bopt_params["n_init_samples"]:
            self.n_init_samples = bopt_params["n_init_samples"]
        else:
            self.n_init_samples = 1
        
        self.gopt_params = gopt_params

        # reset
        self.n_samples = 0
        self.grasps = []
    
    def add_grasp(self, query: "list[float]", outcome: float):
        self.n_samples += 1
        if self.n_samples <= self.n_init_samples:
            it = -1
        else:
            it = self.n_samples - self.n_init_samples
        self.grasps.append([it, query, outcome])

    def get_grasps(self) -> "tuple[list, list]": # init samples, iteration queries
        return self.grasps[:self.n_init_samples], self.grasps[self.n_init_samples:]

    def save_json(self, json_file: str = ""):
        if json_file:
            file = json_file
        elif self.log_file:
            file = self.log_file
        else:
            raise Exception("Error: you must provide a log destination file")
        
        data = {}
        data['bopt_params'] = self.bopt_params
        data['gopt_params'] = {}
        data['gopt_params']['n_grasp_trials'] = self.gopt_params.n_grasp_trials
        data['gopt_params']['active_variables'] = [var_to_str(var) for var in self.gopt_params.active_variables]
        data['gopt_params']['lower_bound'] = list(self.gopt_params.lower_bound)
        data['gopt_params']['upper_bound'] = list(self.gopt_params.upper_bound)
        data['gopt_params']['default_query'] = list(self.gopt_params.default_query)
        data['grasps'] = [{'iteration': g[0], 'query': g[1], 'outcome': g[2]} for g in self.grasps]

        json_str = json.dumps(data, indent=4)
        with open(file, 'w') as outfile:
            # json.dump(json_str, outfile)
            outfile.write(json_str)

    def load_json(self, json_file: str = ""):
        if json_file:
            file = json_file
        elif self.log_file:
            file = self.log_file
        else:
            raise Exception("Error: you must provide a source destination file")

        with open(file, 'r') as f:
            data = json.load(f)

            self.grasps = [[g['iteration'], g['query'], g['outcome']] for g in data['grasps']]
            self.n_samples = len(self.grasps)

            self.bopt_params = data['bopt_params']
            if self.bopt_params["n_init_samples"]:
                self.n_init_samples = self.bopt_params["n_init_samples"]
            else:
                self.n_init_samples = 1
            
            gopt = data['gopt_params']
            n_gras_trials = int(gopt['n_grasp_trials'])
            active_variables = [str_to_var(var) for var in gopt['active_variables']]
            lower_bound = np.array(gopt['lower_bound'], dtype=np.float64)
            upper_bound = np.array(gopt['upper_bound'], dtype=np.float64)
            default_query = np.array(gopt['default_query'], dtype=np.float64)
            self.gopt_params = ActiveGraspingParams(default_query, active_variables, lower_bound, upper_bound, n_gras_trials)


class ActiveGrasping(BayesOptContinuous):
    
    def __init__(self, params: ActiveGraspingParams, bo_params: dict, logger: DataLog = None):
        super().__init__(len(params.active_variables))

        self.default_query: np.ndarray = params.default_query
        self.work_dim: int = params.default_query.shape[0]
        self.active_variables: list[int] = params.active_variables
        self.n_grasp_trials: int = params.n_grasp_trials
        self.executor: GraspExecutor = params.executor

        self.params = bo_params
        self.lower_bound = params.lower_bound
        self.upper_bound = params.upper_bound
        
        self.logger: DataLog = logger
        if self.logger:
            self.logger.setup(params, bo_params)

    def evaluateSample(self, x_in) -> float:
        if self.n_dim < self.work_dim:
            query = self.createOptQuery(x_in)
        else:
            query = x_in
        
        qualities = self.applyQueryToHand(query)

        res = self.evaluateGraspQuality(qualities)
        
        if self.logger:
            self.logger.add_grasp(list(x_in), res)

        return -res

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

        return measure
