from .grasp_models import *
from .metrics import *

def construct_grasp_executor_model(gtype: int, mtype: int = -1, fgrasp: str = "") -> ExecutorModel:
    t = None
    if gtype == 0:
        return GramacyExecutor()
    elif gtype == 1:
        return GraspPlannerExecutor(fgrasp)
    elif gtype == 2:
        return GraspPlannerIKExecutor(fgrasp, mtype)
    else:
        raise Exception("Model " + str(gtype) + " is not valid")

