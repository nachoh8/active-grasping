import argparse
import json
import sigopt

from pygrasp.pygrasp import TestGramacyExecutor

from active_grasping.active_grasping_opt import ActiveGrasping
from active_grasping.datalog import DataLog
from active_grasping.active_grasping_params import *
from active_grasping.sigopt_executor import SigOptExecutor

def sigopt_executor(model: ActiveGrasping, run: sigopt.run_context.RunContext):
    run.log_model("Test Gramacy optimization")
    
    query = [run.params.x1, run.params.x2]
    res = -model.evaluateSample(query)
    
    print("Query:", query, "-> Outcome:", res)

    run.log_metric("outcome", res)
 

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Gramacy optimization')
    parser.add_argument("-fgopt", type=str, help="active grasping params file", metavar='<active_grasp_params_file>', required=True)
    parser.add_argument("-fbopt", type=str, help="bayesopt params file", metavar='<bayesopt_params_file>', default="")
    parser.add_argument("-fsopt", type=str, help="sigopt params file", metavar='<sigopt_params_file>', default="")
    parser.add_argument("-flog", type=str, help="log file", metavar='<log_file>', default="")

    args = parser.parse_args()
    fgopt = args.fgopt
    fbopt = args.fbopt
    fsopt = args.fsopt
    flog = args.flog

    exec_bayes = True
    if fbopt:
        f = open(fbopt, 'r')
        exec_bayes = True
    elif fsopt:
        f = open(fsopt, 'r')
        exec_bayes = False
    else:
        print("Error: you must provide bayesopt params (-fbopt) or sigopt params(-fsopt)")
        exit(-1)
    
    opt_params = json.load(f)
    
    executor = TestGramacyExecutor()

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
        sopt = SigOptExecutor(opt_params, model, sigopt_executor)
        sopt.execute()

    if logger:
        logger.save_json()
        
