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
        
        self.report_failures: bool = False
        if params.get("report_failures"):
            self.report_failures: bool = params["report_failures"]
        self.exp_params: dict = params["exp_params"]
        self.num_runs: int = self.exp_params["budget"]
        self.experiment = sigopt.create_experiment(**self.exp_params)

        default_query = params["default_query"]
        grasp_trials = params["grasp_trials"]
        active_variables = [param["name"] for param in self.exp_params["parameters"]]


        ActiveGrasping.__init__(self, executor, active_variables, default_query, ["outcome", "volume", "force_closure"], n_trials=grasp_trials, logger=logger)

        if self.logger:
            data = {"project": self.project_name, "mode": self.token_type, "report_failures": self.report_failures, "exp_params": self.exp_params}
            self.logger.log_optimizer(params["name"], data)
        
    def run(self):
        print("------------------------")
        print("SIGOPT")
        print("Project: " + self.project_name)
        print("Experiment: " + self.exp_params["name"])
        print("Mode: " + ("dev" if self.token_type == "dev" else "production"))
        print("Active variables: " + str(self.active_variables))
        print("Default query: " + str(self.default_query))
        print("Report failures: " + str(self.report_failures))

        print("Begin experiment")
        print("-------------------------------")
        it = 1
        for run in self.experiment.loop():
            print("----")
            print("Run " + str(it) + "/" + str(self.num_runs))
            with run:
                self.execute_run_query(run)
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
    
    def execute_run_query(self, run: sigopt.run_context.RunContext):
        run.log_model(self.executor.get_name())

        query = dict(run.params)
        res: GraspResult = self.executeQuery(query)

        metrics, errors, metadata = self.executor.parse_results(res)

        not_log_metric = (errors != "" and self.report_failures)

        if not not_log_metric:
            for name, value in metrics:
                run.log_metric(name, value)
        
        for name, value in metadata:
            run.log_metadata(name, value)

        run.log_metadata("default_values", self.executor.get_default_values())

        if errors:
            run.log_metadata("error", errors)
            if self.report_failures:
                run.log_failure()
