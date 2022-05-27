import json

from active_grasping.bayesopt_executor import BayesOptExecutor
from active_grasping.grasp_models import GramacyExecutor

if __name__ == "__main__":
    
    f = open("config/gramacy/tests/bopt_params.json", 'r')
    opt_params = json.load(f)
    f2 = open("config/gramacy/tests/gopt_params.json", 'r')
    gopt_params = json.load(f2)
    
    params = {"bopt_params": opt_params}
    params.update(gopt_params)

    model = BayesOptExecutor(params, GramacyExecutor())
    model.execute()
