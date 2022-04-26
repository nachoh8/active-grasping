import sigopt

from active_grasping.active_grasping_opt import ActiveGrasping
from active_grasping.active_grasping_params import *
from active_grasping.sigopt_executor import SigOptExecutor
from pygrasp.pygrasp import TestGramacyExecutor, GraspResult

PROJECT_NAME = "test-gramacy-sigopt"
NUM_RUNS = 5

def executor(model: ActiveGrasping, run: sigopt.run_context.RunContext):
    run.log_model("Test Gramacy optimization")
    
    query = [run.params.x1, run.params.x2]
    res = -model.evaluateSample(query)
    
    print("Query:", query, "-> Outcome:", res)

    run.log_metric("outcome", res)
    
if __name__ == "__main__":
    # experiment params

    exp_params = dict()
    exp_params["name"] = "Gramacy optimization"
    exp_params["type"] = "offline"
    exp_params["parameters"]=[
        dict(name="x1", type="double", bounds=dict(min=0.0001, max=1.0000), prior=dict(mean=0.5, name="normal", scale=0.2)),
        dict(name="x2", type="double", bounds=dict(min=0.0001, max=1.0000), prior=dict(mean=0.5, name="normal", scale=0.2)),
    ]
    exp_params["metrics" ]= [dict(name="outcome", strategy="optimize", objective="maximize")]
    exp_params["parallel_bandwidth"] = 1
    exp_params["budget"] = NUM_RUNS

    # sigopt params
    sigopt_params = dict()
    sigopt_params["project"] = PROJECT_NAME
    sigopt_params["mode"] = "dev"
    sigopt_params["exp_params"] = exp_params

    # Active grasping params

    default_query = np.zeros((2,))
    active_variables = [0, 1]
    lower_bound = np.zeros((2,))
    upper_bound = np.ones((2,))
    grasp_executor = TestGramacyExecutor()

    params = ActiveGraspingParams(default_query, active_variables, lower_bound, upper_bound, 1, grasp_executor)

    model = ActiveGrasping(params, sigopt_params)

    # Execute experiment
    sopt = SigOptExecutor(sigopt_params, model, executor)
    sopt.execute()
    

    
