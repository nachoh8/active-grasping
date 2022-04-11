import argparse
import matplotlib.pyplot as plt

from pygrasp import *
from active_grasping_opt import *

def outcome_iterations(outcomes: "list[float]", best_acum=False):
    
    iterations = range(1, len(outcomes)+1)
    if best_acum:
        title = 'Best outcome until iteration x'
        Y = [outcomes[0]]
        for i in range(1, len(outcomes)):
            v = outcomes[i] if outcomes[i] > Y[i-1] else Y[i-1]
            Y.append(v)
    else:
        Y = outcomes
        title = 'Value of best selected sample'

    plt.figure()

    plt.plot(iterations, Y, 'ro-')
    plt.xlabel('Iteration')
    plt.ylabel('Outcome')
    plt.title(title)


def distance_queries(queries: list):
    iterations = range(1, len(queries)+1)
    n_vars = len(queries[0])
    matrix = np.array(queries) # #iterations x #vars
    
    total_dist = np.linalg.norm(matrix[:-1] - matrix[1:], axis=1)
    """dist = np.zeros((len(queries) - 1, n_vars + 1))
    dist[:, 1:] = matrix[:-1] - matrix[1:] # var dist
    dist[:, 0] = np.linalg.norm(dist[:,1:], axis=1) # total dist"""
    
    plt.figure()
    plt.plot(iterations[1:], total_dist, 'bo-')
    plt.xlabel('Iteration')
    plt.ylabel('Distance')
    plt.title('Distance between consecutive queries\'s')
    
    """for j in range(dist.shape[1]):
        plt.subplot(1, n_vars+1, j+1)
        plt.plot(iterations[1:], dist[:, j], 'bo-')
    
        #plt.plot(iterations[1:], dist[], 'go-')
        plt.xlabel('Iteration')
        plt.ylabel('Distance')
        plt.title('Distance between consecutive queries\'s')"""


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Active Grasping with bayesian optimization')
    parser.add_argument("-flog", type=str, help="log file", metavar='<log_file>', required=True)

    args = parser.parse_args()
    flog = args.flog

    logger = DataLog(log_file=flog)
    logger.load_json()

    init_samples, samples = logger.get_grasps()
    queries = [s[1] for s in samples]
    outcomes = [s[2] for s in samples]
    
    outcome_iterations(outcomes)

    outcome_iterations(outcomes, best_acum=True)

    distance_queries(queries)

    plt.show()
