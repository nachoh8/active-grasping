import argparse
import numpy as np
import matplotlib.pyplot as plt

from active_grasping.datalog import DataLog

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
    # n_vars = len(queries[0])
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


def outcome_vars(queries: list, outcomes: "list[float]", var_labels: "list[str]"=None, plot2D=False):
    matrix = np.array(queries)
    Z = outcomes
    
    n_vars = len(queries[0])
    X = matrix[:, 0]
    if n_vars > 1:
        Y = matrix[:, 1]
    
    if not var_labels:
        var_labels = ["x", "y"]

    fig = plt.figure()
    if n_vars == 1:
        plt.scatter(X, Z, alpha=0.5)
        plt.xlabel(var_labels[0])
        plt.ylabel("outcome")
    elif plot2D:
        plt.scatter(X, Y, c=Z, alpha=0.5)
        plt.colorbar()
        plt.xlabel(var_labels[0])
        plt.ylabel(var_labels[1])
    else:
        ax = fig.add_subplot(projection='3d')
        ax.scatter(X, Y, Z, c=Z)
        ax.set_xlabel(var_labels[0])
        ax.set_ylabel(var_labels[1])
        ax.set_zlabel("outcome")
    
    plt.title("Distribution of the outcome")
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Active Grasping with bayesian optimization')
    parser.add_argument("-flog", type=str, help="log file", metavar='<log_file>', required=True)

    args = parser.parse_args()
    flog = args.flog

    logger = DataLog(log_file=flog)
    logger.load_json()

    act_vars = logger.get_active_vars()
    print("Active variables: " + str(act_vars))
    print("Metric: " + str(logger.get_metrics()[0]))

    grasps, outcomes = logger.get_grasps()
    
    outcome_iterations(outcomes)

    outcome_iterations(outcomes, best_acum=True)

    distance_queries(grasps)

    outcome_vars(grasps, outcomes, plot2D=False, var_labels=act_vars)

    plt.show()
