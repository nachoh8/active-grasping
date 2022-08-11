from .grasp_models import *
from .metrics import *

def construct_grasp_executor_model(gtype: int, mtype: int = -1, fgrasp: str = "") -> ExecutorModel:
    t = None
    if gtype == GTYPE_GRAMACY:
        return GramacyExecutor()
    elif gtype == GTYPE_GRASP_PLANNER:
        return GraspPlannerExecutor(fgrasp)
    elif gtype == GTYPE_GRASP_PLANNER_IK:
        return GraspPlannerIKExecutor(fgrasp, mtype)
    elif gtype == GTYPE_GRASP_PLANNER_S:
        return GraspPlannerExecutorS(fgrasp)
    else:
        raise Exception("Model " + str(gtype) + " is not valid")

