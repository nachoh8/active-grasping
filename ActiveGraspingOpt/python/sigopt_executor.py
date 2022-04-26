import os
import sigopt

from .active_grasping_opt import ActiveGrasping

class SigOptExecutor(object):
    def __init__(self, sigopt_params: dict, model: ActiveGrasping, run_func: any) -> None:
        self.project_name: str = sigopt_params["project"]
        sigopt.set_project(self.project_name)

        self.token_type: str = sigopt_params["mode"]
        if self.token_type == "dev":
            os.environ["SIGOPT_API_TOKEN"] = os.environ["SIGOPT_DEV_TOKEN"]
        elif self.token_type == "prod":
            os.environ["SIGOPT_API_TOKEN"] = os.environ["SIGOPT_PROD_TOKEN"]
        else:
            raise Exception("Mode not valid, use dev or prod")
        
        self.exp_params: dict = sigopt_params["exp_params"]
        self.num_runs: int = self.exp_params["budget"]
        self.experiment = sigopt.create_experiment(**self.exp_params)

        self.model = model
        self.run_func: function = run_func
        
    def execute(self):
        print("Project: " + self.project_name)
        print("Mode: " + ("dev" if self.token_type == "dev" else "production"))

        print("Begin experiment")
        print("-------------------------------")
        it = 1
        for run in self.experiment.loop():
            print("----")
            print("Run " + str(it) + "/" + str(self.num_runs))
            with run:
                self.run_func(self.model, run)
            it += 1
        
        print("-------------------------------")
        print("End experiment")
        print("-------------------------------")
        
        best_runs = self.experiment.get_best_runs() # generator type
        best = next(best_runs) # sigopt.objects.TrainingRun type
        print("Best result:")
        print("Query:", list(best.assignments.items())) # sigopt.objects.Assignments = dict[param_name: value]
        print("Metrics:", [(metric, value.value) for metric, value in best.values.items()]) # dict[metric_name: sigopt.objects.MetricEvaluation]
            
