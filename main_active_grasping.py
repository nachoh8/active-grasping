import numpy as np
import argparse
import json
import sigopt

from pygrasp.pygrasp import *
from pygrasp.gutils import construct_grasp_executor_int

from active_grasping.active_grasping_opt import ActiveGrasping
from active_grasping.active_grasping_params import *
from active_grasping.datalog import DataLog
from active_grasping.sigopt_executor import SigOptExecutor

def sigopt_executor_grasp(model: ActiveGrasping, run: sigopt.run_context.RunContext):
    run.log_model("Active Grasping optimization")
    
    params = run.params
    query = np.zeros((model.n_dim))
    for pk in params:
        vk = str_to_var(pk)
        idx = model.active_variables.index(vk)
        query[idx] = params[pk]

    res = -model.evaluateSample(query)
    
    print("Query:", query, "-> Outcome:", res)

    run.log_metric("outcome", res)

def sigopt_executor_gramacy(model: ActiveGrasping, run: sigopt.run_context.RunContext):
    run.log_model("Test Gramacy optimization")
    
    query = [run.params.x1, run.params.x2]
    res = -model.evaluateSample(query)
    
    print("Query:", query, "-> Outcome:", res)

    run.log_metric("outcome", res)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Active Grasping with bayesian optimization')
    parser.add_argument("-fgopt", type=str, help="active grasping params file", metavar='<active_grasp_params_file>', required=True)
    parser.add_argument("-fgrasp", nargs=2, help="grasp executor params", metavar=('<executor>', '<params_file>'), required=True)
    parser.add_argument("-fopt", nargs=2, help="optimizer params", metavar=('<optimizer>', '<params_file>'), required=True)
    parser.add_argument("-flog", type=str, help="log file", metavar='<log_file>', default="")

    args = parser.parse_args()
    fgopt = args.fgopt
    fgrasp = [int(args.fgrasp[0]), args.fgrasp[1]]
    fopt = [int(args.fopt[0]), args.fopt[1]]
    flog = args.flog

    if fgrasp[0] > 2:
        print("Error: executor must be {0: TestGramacyExecutor, 1: GraspPlanner, 2: GraspPlannerIK}")
        exit(-1)

    if fopt[0] > 1:
        print("Error: optimizer must be {0: BayesOpt, 1: SigOpt}")
        exit(-1)

    exec_bayes = fopt[0] == 0
    f = open(fopt[1], 'r')
    opt_params = json.load(f)

    executor = construct_grasp_executor_int(int(fgrasp[0]), fgrasp[1])

    gopt_params = load_ActiveGraspingParams(fgopt, executor)

    if flog:
        logger = DataLog(log_file=flog)
    else:
        logger = None

    model = ActiveGrasping(gopt_params, opt_params, logger=logger)

    if exec_bayes:
        print("USING BAYESOPT")
        quality, x_out, _ = model.optimize()

        print("------------------------")
        print("Best:")
        print("\tPoint:", x_out)
        print("\tOutcome:", quality)
    else:
        print("USING SIGOPT")
        func = sigopt_executor_gramacy if fgrasp[0] == 0 else sigopt_executor_grasp
        sopt = SigOptExecutor(opt_params, model, func)
        sopt.execute()

    if logger:
        logger.save_json()
        
