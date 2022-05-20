import json

from active_grasping.sigopt_executor import SigOptExecutor
from active_grasping.grasp_models import GramacyExecutor
    
if __name__ == "__main__":
    # experiment params

    """exp_params = dict()
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
    sigopt_params["default_query"] = {"x1": 0, "x2": 0}
    sigopt_params["grasp_trials"] = 1
    sigopt_params["exp_params"] = exp_params"""

    # Active grasping params

    f = open("config/gramacy/tests/sigopt_params.json", 'r')
    sigopt_params = json.load(f)

    model = SigOptExecutor(sigopt_params, GramacyExecutor())

    model.execute()
    

    
