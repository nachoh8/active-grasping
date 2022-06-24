import json
from datetime import datetime

from pygrasp.pygrasp import GraspResult


BASIC_PARAMS_KEY = "basic_params"
GRASP_EXECUTOR_KEY = "grasp_executor"
OPTIMIZER_KEY = "optimizer"
BEST_RESULT_KEY = "best_result"
GRASPS_KEY = "grasps"

class DataLog(object):
    def __init__(self, log_file: str = "") -> None:
        self.log_file: str = log_file
        self.data = dict()
        self.data["date"] = datetime.now().strftime("%d/%m/%Y %H:%M:%S")

        self.basic_params: dict = dict()
        self.grasp_executor: dict = dict()
        self.optimizer: dict = dict()
        self.best_results: list[dict] = []
        self.grasps: list[dict] = []
    
    def log(self, key: any, data: any):
        self.data[key] = data
    
    def log_basic_params(self, params: dict):
        self.log(BASIC_PARAMS_KEY, params)
    
    def log_grasp_executor(self, name: str, params: dict):
        self.grasp_executor = {"name": name, "params": params}
        self.log(GRASP_EXECUTOR_KEY, self.grasp_executor)
    
    def log_optimizer(self, name: str, params: dict):
        self.optimizer = {"name": name, "params": params}
        self.log(OPTIMIZER_KEY, self.optimizer)
    
    def log_best_results(self, results: "list[dict]"):
        self.best_results = results
        self.log(BEST_RESULT_KEY, self.best_results)

    def log_grasps(self, grasps: "list[dict]"):
        self.grasps = grasps
        self.log(GRASPS_KEY, self.grasps)

    def save_json(self, json_file: str = ""):
        if json_file:
            file = json_file
        elif self.log_file:
            file = self.log_file
        else:
            raise Exception("Error: you must provide a log destination file")
        
        json_str = json.dumps(self.data, indent=4)
        with open(file, 'w') as outfile:
            outfile.write(json_str)

    def load_json(self, json_file: str = ""):
        if json_file:
            file = json_file
        elif self.log_file:
            file = self.log_file
        else:
            raise Exception("Error: you must provide a log file")

        with open(file, 'r') as f:
            self.data = json.load(f)

            self.basic_params   = self.data[BASIC_PARAMS_KEY]
            self.grasp_executor = self.data[GRASP_EXECUTOR_KEY]
            self.optimizer      = self.data[OPTIMIZER_KEY]
            self.best_results   = self.data[BEST_RESULT_KEY]
            self.grasps         = self.data[GRASPS_KEY]
    
    def get_optimizer_name(self) -> str:
        return self.optimizer["name"]

    def get_active_vars(self) -> "list[str]":
        return self.basic_params["active_variables"]
    
    def get_metrics(self) -> "list[str]":
        return self.basic_params["metrics"]

    def get_grasps(self, metric: str = "outcome") -> "tuple[list, list]":
        act_vars = self.get_active_vars()
        n_var = len(act_vars)
        
        grasps = []
        metrics = []
        for data_grasp in self.grasps:
            grasp = [None] * n_var
            
            q = data_grasp["query"]
            for var in act_vars:
                idx = act_vars.index(var)
                grasp[idx] = q[var]
            
            grasps.append(grasp)

            metrics.append(data_grasp["metrics"][metric])
        
        return grasps, metrics
    
    def get_rho(self, metric: str = "computed_rho") -> "tuple[list]":
        metrics = []
        for data_grasp in self.grasps:

            metrics.append(data_grasp["metrics"][metric])
        
        return metrics

    def get_best_grasps(self, metric: str = "outcome") -> "tuple[list, list]":
        """

        """
        act_vars = self.get_active_vars()
        n_var = len(act_vars)
        
        grasps = []
        metrics = []
        for data_grasp in self.best_results:
            grasp = [None] * n_var
            
            q = data_grasp["query"]
            for var in act_vars:
                idx = act_vars.index(var)
                grasp[idx] = q[var]
            
            grasps.append(grasp)
            print(data_grasp["metrics"])
            for m in data_grasp["metrics"]:
                if m["name"] == metric:
                    metrics.append(m['value'])
        
        return grasps, metrics