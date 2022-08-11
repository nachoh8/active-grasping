from pygrasp.pygrasp import GraspResult

from .datalog import DataLog
from .grasp_models import ExecutorModel, GraspPlannerIKExecutor

class ActiveGrasping:
    def __init__(self, executor: ExecutorModel, active_variables: "list[str]", default_query: dict, metrics: "list[str]",
                    n_trials: int = 1, logger: DataLog = None) -> None:
        self.active_variables: list[int] = active_variables
        self.default_query: dict = default_query
        self.opt_dim: int = len(self.active_variables)
        self.work_dim: int = len(self.default_query)
        self.n_trials: int = n_trials
        self.metrics = metrics
        
        self.executor: ExecutorModel = executor
        self.executor.set_default_values(self.default_query)

        self.queries = []
        self.best_results = []

        self.logger: DataLog = logger
        if self.logger:
            params_log = {"active_variables": self.active_variables, "default_query": self.default_query, "metrics": self.metrics, "n_trials": self.n_trials}
            self.logger.log_basic_params(params_log)
            self.logger.log_grasp_executor(self.executor.get_name(), self.executor.get_params())

    def executeQuery(self, query: dict) -> GraspResult:
        values = self.executor.query_to_values(query)
        ok = False
        i = 0
        while i < self.n_trials and not ok:
            res: GraspResult = self.executor.execute(values)
            ok = res.error == ""
            i +=1

        log_data = self.executor.metric_parser.get_data_log(res)
        if log_data.get("others"):
            log_data["others"]["trials"] = i
        else:
            log_data["others"] = {"trials": i}

        if res.error != "":
            print("Query:", query, "-> Error:", res.error, " trials:", i)
        else:
            print("Query:", query)
            print(log_data)
        
        log_data["query"] = query

        self.queries.append(log_data)

        return res
    
    def evaluateQueryTrials(self, qualities: "list[GraspResult]") -> list:
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
    
    def run(self):
        raise Exception("This is an abstract class")

    def execute(self):
        self.run()
        if self.logger:
            self.logger.log_best_results(self.best_results)
            self.logger.log_grasps(self.queries)
            self.logger.save_json()
