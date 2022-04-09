import sys
import numpy as np
import argparse
import json

from pygrasp import *
from active_grasping_opt import *

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Active Grasping with bayesian optimization')
    parser.add_argument("--fgopt", type=str, help="active grasping params file", metavar='<active_grasp_params_file>', required=True)
    parser.add_argument("--fbopt", type=str, help="bayesopt params file", metavar='<bayesopt_params_file>', default="")
    parser.add_argument("--fgrasp", type=str, help="grasp planner params file", metavar='<grasp_planner_params_file>', required=True)

    args = parser.parse_args()
    fgopt = args.fgopt
    fbopt = args.fbopt
    fgrasp = args.fgrasp

    if fbopt:
        f = open(fbopt, 'r')
        bopt_params = json.load(f)
    else:
        bopt_params = {}

    grasp_params = GraspPlannerParams()
    if not load_GraspPlannerParams_json(fgrasp, grasp_params):
        print("Error: parsing grasp planner params")
        exit(1)
    
    executor = GraspPlanner(grasp_params)

    gopt_params = load_ActiveGraspingParams(fgopt, executor)

    optimizer = ActiveGrasping(gopt_params, bopt_params)

    mvalue, x_out, error = optimizer.optimize()

    print("Result:", x_out)
    print("Mvalue:", mvalue)
    print("Error:", error)

    
