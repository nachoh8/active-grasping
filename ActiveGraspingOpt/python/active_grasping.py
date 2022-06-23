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
        res: GraspResult = self.executor.execute(values)
        
        print("Query:", query, "-> Outcome:", res.measure, "Volume:", res.volume, "Force closure:", res.force_closure)
        
        log_data = {"query": query, "metrics": {"outcome": res.measure, "volume": res.volume, "force_closure": res.force_closure, "computed_rho": res.rho}}
        if type(self.executor) == GraspPlannerIKExecutor:
            log_data.update({"others":{"time": res.time, "position_error": res.pos_error, "orientation_error": res.ori_error}})
            print("Time:", res.time, "Position Error:", res.pos_error, "Orientation Error:", res.ori_error)
        
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
