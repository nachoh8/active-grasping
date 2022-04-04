import numpy as np

from pygrasp import *
from bayesoptmodule import BayesOptContinuous

class ActiveGraspingParams:
    def __init__(self, default_query, active_variables, lower_bound, upper_bound, executor):
        self.default_query = default_query
        self.active_variables = active_variables
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.executor = executor

class ActiveGrasping(BayesOptContinuous):
    def __init__(self, params, bo_params):
        super(ActiveGrasping, self).__init__(len(params.active_variables))
        self.my_params = params
        self.work_dim = params.default_query.shape[0]

        self.params = bo_params
        self.lower_bound = params.lower_bound
        self.upper_bound = params.upper_bound

        self.executor = TestGramacyExecutor()

    def evaluateSample(self, x_in):
        res = self.my_params.executor.executeQueryGrasp(x_in) # GraspResult
        
        return res.measure

    ### PRIVATE METHODS

    def createOptQuery(self, query):
        pass

    def applyQueryToHand(self, query):
        pass

    def evaluateGraspQuality(self, qualities):
        pass


default_query = np.zeros((2,))
active_variables = [0, 1]
lower_bound = np.zeros((2,))
upper_bound = np.ones((2,))
executor = TestGramacyExecutor()

params = ActiveGraspingParams(default_query, active_variables, lower_bound, upper_bound, executor)

opt = ActiveGrasping(params, {})

mvalue, x_out, error = opt.optimize()
print("Result:", x_out)
print("Mvalue:", mvalue)
print("Error:", error)
