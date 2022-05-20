import os
import sigopt

from pygrasp.pygrasp import GraspResult

from .active_grasping import ActiveGrasping
from .grasp_models import *
from .datalog import DataLog

class SigOptExecutor(ActiveGrasping):
    def __init__(self, params: dict, executor: ExecutorModel, logger: DataLog = None) -> None:

        self.project_name: str = params["project"]
        sigopt.set_project(self.project_name)

        self.token_type: str = params["mode"]
        if self.token_type == "dev":
            os.environ["SIGOPT_API_TOKEN"] = os.environ["SIGOPT_DEV_TOKEN"]
        elif self.token_type == "prod":
            os.environ["SIGOPT_API_TOKEN"] = os.environ["SIGOPT_PROD_TOKEN"]
        else:
            raise Exception("Mode not valid, use dev or prod")
        
        self.exp_params: dict = params["exp_params"]
        self.num_runs: int = self.exp_params["budget"]
        self.experiment = sigopt.create_experiment(**self.exp_params)

        default_query = params["default_query"]
        grasp_trials = params["grasp_trials"]
        active_variables = [param["name"] for param in self.exp_params["parameters"]]


        ActiveGrasping.__init__(self, executor, active_variables, default_query, ["outcome", "volume", "force_closure"], n_trials=grasp_trials, logger=logger)

        self.run_func: function = None
        if type(self.executor) == GramacyExecutor:
            self.run_func = sigopt_executor_gramacy
        else:
            self.run_func = sigopt_executor_grasp

        if self.logger:
            data = {"project": self.project_name, "mode": self.token_type, "exp_params": self.exp_params}
            self.logger.log_optimizer("sigopt", data)
        
    def run(self):
        print("------------------------")
        print("SIGOPT")
        print("Project: " + self.project_name)
        print("Experiment: " + self.exp_params["name"])
        print("Mode: " + ("dev" if self.token_type == "dev" else "production"))
        print("Active variables: " + str(self.active_variables))
        print("Default query: " + str(self.default_query))

        print("Begin experiment")
        print("-------------------------------")
        it = 1
        for run in self.experiment.loop():
            print("----")
            print("Run " + str(it) + "/" + str(self.num_runs))
            with run:
                self.run_func(self, run)
            it += 1
        
        print("-------------------------------")
        print("End experiment")
        print("-------------------------------")
        
        best_runs = self.experiment.get_best_runs() # generator type
        print("Best results:")
        self.best_results = []
        for run in best_runs: # sigopt.objects.TrainingRun type
            print("Query:", list(run.assignments.items())) # sigopt.objects.Assignments = dict[param_name: value]
            print("Metrics:", [(metric, value.value) for metric, value in run.values.items()]) # dict[metric_name: sigopt.objects.MetricEvaluation]
            
            r = {"query": run.assignments, "metrics": [{"name": metric, "value": value.value} for metric, value in run.values.items()]}
            self.best_results.append(r)

            
def sigopt_executor_grasp(sigopt_executor: SigOptExecutor, run: sigopt.run_context.RunContext):
    run.log_model(sigopt_executor.executor.get_name())
    
    query = run.params
    res: GraspResult = sigopt_executor.executeQuery(query)

    log_query = [None] * sigopt_executor.opt_dim
    for pk in query:
        idx = sigopt_executor.active_variables.index(pk)
        log_query[idx] = query[pk]

    print("Query:", query, "-> Outcome:", res.measure, "Volume:", res.volume, "Force closure:", res.force_closure)

    run.log_metric("outcome", res.measure)
    run.log_metadata("volume", res.volume)
    run.log_metadata("force_closure", res.force_closure)
    run.log_metadata("default_values", sigopt_executor.executor.get_default_values())

    log_data = {"query": log_query, "metrics": [res.measure, res.volume, res.force_closure]}
    if type(sigopt_executor.executor) == GraspPlannerIKExecutor:
        print("Time:", res.time, "Position Error:", res.pos_error, "Orientation Error:", res.ori_error)
        run.log_metadata("time(ms)", res.time)
        run.log_metadata("position_error(mm)", res.pos_error)
        run.log_metadata("orientation_error(degrees)", res.ori_error)
        log_data.update({"others":{"time": res.time, "position_error": res.pos_error, "orientation_error": res.ori_error}})
    
    sigopt_executor.queries.append(log_data)

def sigopt_executor_gramacy(sigopt_executor: SigOptExecutor, run: sigopt.run_context.RunContext):
    run.log_model("Test Gramacy")
    
    query = run.params
    res: GraspResult = sigopt_executor.executeQuery(query)

    log_query = [None] * sigopt_executor.opt_dim
    for pk in query:
        idx = sigopt_executor.active_variables.index(pk)
        log_query[idx] = query[pk]

    sigopt_executor.queries.append({"query": log_query, "metrics": [res.measure, res.volume, res.force_closure]})
    
    print("Query:", query, "-> Outcome:", res.measure)

    run.log_metric("outcome", res.measure)
