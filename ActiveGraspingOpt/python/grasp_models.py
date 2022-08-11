import json
from pygrasp.pygrasp import *

from .metrics import *

GTYPE_GRAMACY = 0
GTYPE_GRASP_PLANNER = 1
GTYPE_GRASP_PLANNER_IK = 2
GTYPE_GRASP_PLANNER_S = 3

def construct_grasp_executor_int(gtype: int, fgrasp: str = "") -> GraspExecutor:
    t = None
    if gtype == GTYPE_GRAMACY:
        t = TestGramacyExecutor
    elif gtype == GTYPE_GRASP_PLANNER:
        t = GraspPlanner
    elif gtype == GTYPE_GRASP_PLANNER_IK:
        t = GraspPlannerIK
    elif gtype == GTYPE_GRASP_PLANNER_S:
        t = GraspPlannerS
    
    return construct_grasp_executor(t, fgrasp)

def construct_grasp_executor(gtype: type, fgrasp: str = "") -> GraspExecutor:
    if gtype == TestGramacyExecutor:
        return TestGramacyExecutor(), None
    elif gtype == GraspPlanner:
        grasp_params = GraspPlannerParams()
        if not load_GraspPlannerParams_json(fgrasp, grasp_params):
            print("Error: parsing grasp planner params")
            return None
        return GraspPlanner(grasp_params), grasp_params
    elif gtype == GraspPlannerIK:
        grasp_params = GraspPlannerIKParams()
        if not load_GraspPlannerIKParams(fgrasp, grasp_params):
            print("Error: parsing grasp planner ik params")
            return None
        return GraspPlannerIK(grasp_params), grasp_params
    elif gtype == GraspPlannerS:
        grasp_params = GraspPlannerParams()
        if not load_GraspPlannerParams_json(fgrasp, grasp_params):
            print("Error: parsing grasp planner params")
            return None
        return GraspPlannerS(grasp_params), grasp_params
    else:
        return None

def cartesian_var_to_idx(var: str) -> int:
    if var == "x":
        return 0
    elif var == "y":
        return 1
    elif var == "z":
        return 2
    elif var == "rx":
        return 3
    elif var == "ry":
        return 4
    elif var == "rz":
        return 5
    else:
        raise Exception("Variable " + var + " is not valid")

def cartesian_idx_to_var(idx: int) -> str:
    if idx == 0:
        return "x"
    elif idx == 1:
        return "y"
    elif idx == 2:
        return "z"
    elif idx == 3:
        return "rx"
    elif idx == 4:
        return "ry"
    elif idx == 5:
        return "rz"
    else:
        raise Exception("Index " + str(idx) + " is not valid")

def spherical_var_to_idx(var: str) -> int:
    if var == "theta":
        return 0
    elif var == "phi":
        return 1
    elif var == "rho":
        return 2
    elif var == "rx":
        return 3
    elif var == "ry":
        return 4
    elif var == "rz":
        return 5
    else:
        raise Exception("Variable " + var + " is not valid")

def spherical_idx_to_var(idx: int) -> str:
    if idx == 0:
        return "theta"
    elif idx == 1:
        return "phi"
    elif idx == 2:
        return "rho"
    elif idx == 3:
        return "rx"
    elif idx == 4:
        return "ry"
    elif idx == 5:
        return "rz"
    else:
        raise Exception("Index " + str(idx) + " is not valid")

class ExecutorModel(object):
    def __init__(self, values_size: int, gtype: type, metric_parser: GraspMetric, fparams: str = "") -> None:
        self.executor: GraspExecutor = None
        self.params: any = None
        self.executor, self.params = construct_grasp_executor(gtype, fparams)

        self.values_size = values_size
        self.default_values = [None] * values_size

        self.metric_parser: GraspMetric = metric_parser
    
    def var_to_idx(self, var: str) -> int:
        raise Exception("This is an abstract class")

    def idx_to_var(self, idx: int) -> str:
        raise Exception("This is an abstract class")
    
    def query_to_values(self, query: dict) -> list:
        n = len(query)
        if n == 0 or n > self.values_size:
            raise Exception("Query size must be 1 <= size <= " + str(self.values_size))
        
        values = self.default_values.copy()
        for k,v in list(query.items()):
            idx = self.var_to_idx(k)
            values[idx] = v
        
        return values

    def set_default_values(self, values: dict):
        if len(values) != self.values_size:
            raise Exception("Default values size must be " + str(self.values_size))
        
        self.default_values = self.query_to_values(values)

    def get_params(self) -> dict:
        return dict()
    
    def get_name(self) -> str:
        raise Exception("This is an abstract class")
    
    def get_default_values(self) -> list:
        return self.default_values
    
    def execute(self, values: list) -> GraspResult:
        if len(values) != self.values_size:
            raise Exception("Values size must be " + str(self.values_size))

        return self.executor.executeQueryGrasp(values)

    def parse_results(self, res: GraspResult, is_sigopt: bool) -> "tuple[dict, any]":
        """
        return
        - data to local log
        - data to evaluate with bayesopt/sigopt
        """

        if is_sigopt:
            return self.metric_parser.get_data_log(res), self.metric_parser.get_sigopt_data(res)
        else:
            return self.metric_parser.get_data_log(res), self.metric_parser.get_bayesopt_metric(res)

class GramacyExecutor(ExecutorModel):
    def __init__(self) -> None:
        super().__init__(2, TestGramacyExecutor, GramacyMetric()), 

    def var_to_idx(self, var: str) -> int:
        if var == "x1":
            return 0
        elif var == "x2":
            return 1
        else:
            raise Exception("Variable " + var + " is not valid")
    
    def idx_to_var(self, idx: int) -> str:
        if idx == 0:
            return "x1"
        elif idx == 1:
            return "x2"
        else:
            raise Exception("Index " + str(idx) + " is not valid")

    def get_name(self) -> str:
        return "Gramacy"
    
class GraspPlannerExecutor(ExecutorModel):
    def __init__(self, fgrasp: str) -> None:
        super().__init__(6, GraspPlanner, ForceClosure(), fgrasp)
        f = open(fgrasp, 'r')
        self.json_params = json.load(f)

    def var_to_idx(self, var: str) -> int:
        return cartesian_var_to_idx(var)
    
    def idx_to_var(self, idx: int) -> str:
        return cartesian_idx_to_var(idx)
    
    def get_name(self) -> str:
        return "GraspPlanner"
    
    def get_params(self) -> dict:
        return self.json_params
    
class GraspPlannerIKExecutor(ExecutorModel):
    def __init__(self, fgrasp: str, metric_type: int) -> None:
        if metric_type == MTYPE_FC_IK:
            metric = ForceClosureIK()
        elif metric_type == MTYPE_FC_T_IK:
            metric = ForceClosureTimeIK()
        else:
            raise Exception("Metric not compatible")
        
        super().__init__(6, GraspPlannerIK, metric, fgrasp)
        f = open(fgrasp, 'r')
        self.json_params = json.load(f)

    def var_to_idx(self, var: str) -> int:
        return cartesian_var_to_idx(var)
    
    def idx_to_var(self, idx: int) -> str:
        return cartesian_idx_to_var(idx)
    
    def get_name(self) -> str:
        return "GraspPlannerIK"
    
    def get_params(self) -> dict:
        return self.json_params

class GraspPlannerExecutorS(ExecutorModel):
    def __init__(self, fgrasp: str) -> None:
        super().__init__(6, GraspPlannerS, ForceClosureS(), fgrasp)
        f = open(fgrasp, 'r')
        self.json_params = json.load(f)

    def var_to_idx(self, var: str) -> int:
        return spherical_var_to_idx(var)
    
    def idx_to_var(self, idx: int) -> str:
        return spherical_idx_to_var(idx)
    
    def get_name(self) -> str:
        return "GraspPlannerS"
    
    def get_params(self) -> dict:
        return self.json_params
