import sys
import numpy as np

from pygrasp.pygrasp import *

from active_grasping.active_grasping_opt import ActiveGrasping
from active_grasping.active_grasping_params import *


def test_gramacy() -> ActiveGrasping:

    default_query = np.zeros((2,))
    active_variables = [0, 1]
    lower_bound = np.zeros((2,))
    upper_bound = np.ones((2,))
    executor = TestGramacyExecutor()

    params = ActiveGraspingParams(default_query, active_variables, lower_bound, upper_bound, 1, executor)

    opt = ActiveGrasping(params, {})

    return "Gramacy", opt

def test_simox() -> ActiveGrasping:
    grasp_params = GraspPlannerParams()
    is_valid = load_GraspPlannerParams_json("/home/juangls/ActiveGrasping/active-grasping/config/grasp/grasp_params.json", grasp_params) # or from file
    if not is_valid:
        print("Error: parsing gras planner params")
        exit(1)
    
    grasp_params.has_obj_pose = True
    grasp_params.obj_position = np.ndarray((3,1), buffer=np.array([93, 34, 45]), dtype=np.float32)
    grasp_params.obj_orientation = np.ndarray((3,1), buffer=np.array([1.4, 2.84, -3.1]), dtype=np.float32)

    executor = GraspPlanner(grasp_params)
    opt_params = load_ActiveGraspingParams("/home/juangls/ActiveGrasping/active-grasping/config/grasp/gopt_z.json", executor)

    opt = ActiveGrasping(opt_params, {})

    return "SIMOX", opt

def exec_opt(name: str, opt: ActiveGrasping):
    print("---TEST " + name + "---")

    mvalue, x_out, error = opt.optimize()

    print("Result:", x_out)
    print("Mvalue:", mvalue)
    print("Error:", error)

if __name__ == "__main__":
    if len(sys.argv) == 2: # 0: gramacy, 1: simox, 2: all
        test = int(sys.argv[1])
    else:
        test = 2
    
    opts = []
    if test == 0 or test == 2:
        opts.append(test_gramacy())
    
    if test == 1 or test == 2:
        opts.append(test_simox())
    
    for opt in opts:
        exec_opt(opt[0], opt[1])
