import argparse
import matplotlib.pyplot as plt

from pygrasp import *
from active_grasping_opt import *

from ActiveGraspingOpt.python.active_grasping_opt import DataLog

def outcome_iterations(outcomes: "list[float]"):
    iterations = range(1, len(outcomes)+1)

    plt.plot(iterations, outcomes, 'ro-')
    plt.xlabel('Iteration')
    plt.ylabel('Outcome')
    plt.title('Value of best selected sample')

def distance_queries(queries: list):
    iterations = range(1, len(queries)+1)
    x_neighbor_dist = [np.linalg.norm(np.array(a)-np.array(b)) for a, b in zip(queries, queries[1:])]

    plt.plot(iterations[1:], x_neighbor_dist, 'bo-')
    plt.xlabel('Iteration')
    plt.ylabel('Distance')
    plt.title('Distance between consecutive queries\'s')


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
    
    plt.figure(0)
    outcome_iterations(outcomes)

    plt.figure(1)
    distance_queries(queries)

    plt.show()
