import numpy as np
import json

from pygrasp.pygrasp import GraspResult
from pygrasp.gutils import str_to_var, var_to_str

from .active_grasping_params import ActiveGraspingParams

class DataLog(object):
    def __init__(self, log_file: str = "") -> None:
        self.gopt_params: ActiveGraspingParams = None
        self.opt_params: dict = {}
        self.n_init_samples = 0
        self.n_samples: int = 0
        self.grasps = []
        self.log_file: str = log_file

    def setup(self, gopt_params: ActiveGraspingParams, opt_params: dict):
        self.opt_params = opt_params
        if opt_params.get("n_init_samples"):
            self.n_init_samples = opt_params["n_init_samples"]
        else:
            self.n_init_samples = 0
        
        self.gopt_params = gopt_params

        # reset
        self.n_samples = 0
        self.grasps = []
    
    def add_grasp(self, query: "list[float]", res: GraspResult):
        self.n_samples += 1
        if self.n_samples <= self.n_init_samples:
            it = -1
        else:
            it = self.n_samples - self.n_init_samples
        self.grasps.append([it, query, res])

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
        data['opt_params'] = self.opt_params
        data['gopt_params'] = {}
        data['gopt_params']['n_grasp_trials'] = self.gopt_params.n_grasp_trials
        data['gopt_params']['active_variables'] = [var_to_str(var) for var in self.gopt_params.active_variables]
        data['gopt_params']['lower_bound'] = list(self.gopt_params.lower_bound)
        data['gopt_params']['upper_bound'] = list(self.gopt_params.upper_bound)
        data['gopt_params']['default_query'] = list(self.gopt_params.default_query)
        data['grasps'] = [{'iteration': g[0], 'query': g[1], 'res': {'measure': g[2].measure, 'volume': g[2].volume, 'force_closure': g[2].force_closure}} for g in self.grasps]

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
            raise Exception("Error: you must provide a log source file")

        with open(file, 'r') as f:
            data = json.load(f)

            self.grasps = [[g['iteration'], g['query'], GraspResult(g['res']['measure'], g['res']['volume'], g['res']['force_closure'])] for g in data['grasps']]
            
            self.n_samples = len(self.grasps)

            self.opt_params = data['opt_params']
            if self.opt_params.get("n_init_samples"):
                self.n_init_samples = self.opt_params["n_init_samples"]
            else:
                self.n_init_samples = 0
            
            gopt = data['gopt_params']
            n_gras_trials = int(gopt['n_grasp_trials'])
            active_variables = [str_to_var(var) for var in gopt['active_variables']]
            lower_bound = np.array(gopt['lower_bound'], dtype=np.float64)
            upper_bound = np.array(gopt['upper_bound'], dtype=np.float64)
            default_query = np.array(gopt['default_query'], dtype=np.float64)
            self.gopt_params = ActiveGraspingParams(default_query, active_variables, lower_bound, upper_bound, n_gras_trials)
