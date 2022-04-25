import numpy as np
import argparse
import json

from pygrasp import *
from active_grasping_opt import *

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Active Grasping with bayesian optimization')
    parser.add_argument("-fgopt", type=str, help="active grasping params file", metavar='<active_grasp_params_file>', required=True)
    parser.add_argument("-fbopt", type=str, help="bayesopt params file", metavar='<bayesopt_params_file>', default="")
    parser.add_argument("-flog", type=str, help="log file", metavar='<log_file>', default="")

    args = parser.parse_args()
    fgopt = args.fgopt
    fbopt = args.fbopt
    flog = args.flog

    if fbopt:
        f = open(fbopt, 'r')
        bopt_params = json.load(f)
    else:
        bopt_params = {}
    
    executor = TestGramacyExecutor()

    gopt_params = load_ActiveGraspingParams(fgopt, executor)
    
    if flog:
        logger = DataLog(log_file=flog)
    else:
        logger = None

    optimizer = ActiveGrasping(gopt_params, bopt_params, logger=logger)

    quality, x_out, _ = optimizer.optimize()

    print("------------------------")
    print("Best:")
    print("\tPoint:", x_out)
    print("\tOutcome:", quality)

    if logger:
        logger.save_json()
        
