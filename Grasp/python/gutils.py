from .pygrasp import *

def construct_grasp_executor(gtype: type, fgrasp: str = "") -> GraspExecutor:
    if gtype == TestGramacyExecutor:
        return TestGramacyExecutor()
    elif gtype == GraspPlanner:
        grasp_params = GraspPlannerParams()
        if not load_GraspPlannerParams_json(fgrasp, grasp_params):
            print("Error: parsing grasp planner params")
            return None
        return GraspPlanner(grasp_params)
    else:
        return None

def str_to_var(var: str) -> int:
    """""
    if var == "x":
        return TRANS_X
    elif var == "y":
        return TRANS_Y
    elif var == "z":
        return TRANS_Z
    """
    if var == "theta":
        return TRANS_THETA
    elif var == "phi":
        return TRANS_PHI
    elif var == "rho":
        return TRANS_RHO
    elif var == "rx":
        return ROT_ROLL
    elif var == "ry":
        return ROT_PITCH
    elif var == "rz":
        return ROT_YAW
    else:
        return var

def var_to_str(var: int) -> str:
    """
    if var == TRANS_X:
        return "x"
    elif var == TRANS_Y:
        return "y"
    elif var == TRANS_Z:
        return "z"
    """
    if var == TRANS_THETA:
        return "theta"
    elif var == TRANS_PHI:
        return "phi"
    elif var == TRANS_RHO:
        return "rho"
    elif var == ROT_ROLL:
        return "rx"
    elif var == ROT_PITCH:
        return "ry"
    elif var == ROT_YAW:
        return "rz"
    else:
        return var

