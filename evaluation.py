import argparse
import os
import glob
import numpy as np
import matplotlib.pyplot as plt

from active_grasping.datalog import DataLog
from pyparsing import alphas

def outcome_iterations(outcomes: "list[float]", best_acum=False, errors: "list[float]" = None):
    
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

    if errors:
        plt.errorbar(iterations, Y, yerr=errors, fmt='o')
    else:
        plt.plot(iterations, Y, 'ro-')
    plt.xlabel('Iteration')
    plt.ylabel('Outcome')
    plt.title(title)


def distance_queries(queries: np.ndarray):
    iterations = range(1, queries.shape[0]+1)
    total_dist = np.linalg.norm(queries[:-1] - queries[1:], axis=1)
    
    plt.figure()
    plt.plot(iterations[1:], total_dist, 'bo-')
    plt.xlabel('Iteration')
    plt.ylabel('Distance')
    plt.title('Distance between consecutive queries\'s')


def outcome_vars(queries: np.ndarray, outcomes: "list[float]", var_labels: "list[str]", plot2D=False):
    n_vars = len(var_labels)

    fig = plt.figure()
    if n_vars == 1:
        sc_plot = plt.scatter(queries[:, 0], outcomes, c=outcomes, alpha=0.5)
        plt.xlabel(var_labels[0])
        plt.ylabel("outcome")
    elif plot2D:
        sc_plot = plt.scatter(queries[:, 0], queries[:, 1], c=outcomes, alpha=0.5)
        plt.xlabel(var_labels[0])
        plt.ylabel(var_labels[1])
    else:
        ax = fig.add_subplot(projection='3d')
        if n_vars == 2:
            sc_plot = ax.scatter(queries[:, 0], queries[:, 1], outcomes, c=outcomes, alpha=0.5)
            ax.set_zlabel("outcome")
        else:
            sc_plot = ax.scatter(queries[:, 0], queries[:, 1], queries[:, 2], c=outcomes, alpha=0.5)
            ax.set_zlabel(var_labels[2])
        
        ax.set_xlabel(var_labels[0])
        ax.set_ylabel(var_labels[1])

    plt.colorbar(sc_plot, label="outcome", orientation="vertical")
    
    plt.title("Distribution of the outcome")
    plt.show()


def get_values(file_path: str) -> "tuple[list[str], np.ndarray, list[float]]":
    """
    input: log file
    output: active variables, grasps, outcomes
    """

    logger = DataLog(log_file=file_path)
    logger.load_json()

    act_vars = logger.get_active_vars()
    queries, outcomes = logger.get_grasps()

    grasps = np.array(queries)

    return act_vars, grasps, outcomes

def get_average_values(folder_path: str) -> "tuple[list[str], np.ndarray, list[float], list[float], list[float]]":
    """
    input: log file
    output: active variables, all grasps, all outcomes, avg outcomes, std dev
    """

    logs = glob.glob(folder_path + "/*.json")
    print("Num. log files: " + str(len(logs)))

    act_vars, all_grasps, outcomes = get_values(logs[0])
    all_outcomes = np.array(outcomes).reshape(1, -1)
    logs.pop(0)

    for file in logs:
        print("Adding", file)
        _, grasps, outcomes = get_values(logs[0])
        all_grasps = np.append(all_grasps, grasps, axis=0)
        m_outcomes = np.array(outcomes).reshape(1, -1)
        all_outcomes = np.append(all_outcomes, m_outcomes, axis=0)
    
    avg_outcomes = list(np.mean(all_outcomes, axis=0))
    std_dev = list(np.std(all_outcomes, axis=0))
    all_outcomes = list(all_outcomes.flatten())

    return act_vars, all_grasps, all_outcomes, avg_outcomes, std_dev

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Active Grasping with bayesian optimization')
    parser.add_argument("-flog", type=str, help="log file/folder", metavar='<log_file> | <folder>', required=True)

    args = parser.parse_args()
    flog = args.flog

    if os.path.isdir(flog):  
        print("Loading results from " + flog + " folder")
        act_vars, grasps, all_outcomes, outcomes, std_dev = get_average_values(flog)

        print("Active variables: " + str(act_vars))
        print("Num. total grasps: " + str(grasps.shape[0]))

        outcome_iterations(outcomes, errors=std_dev)

        outcome_iterations(outcomes, best_acum=True, errors=std_dev)

        outcome_vars(grasps, all_outcomes, plot2D=False, var_labels=act_vars)
    else:  
        print("Loading results from " + flog + " file")
        act_vars, grasps, outcomes = get_values(flog)

        print("Active variables: " + str(act_vars))
        print("Num. grasps: " + str(grasps.shape[0]))
    
        outcome_iterations(outcomes)

        outcome_iterations(outcomes, best_acum=True)

        distance_queries(grasps)

        outcome_vars(grasps, outcomes, plot2D=False, var_labels=act_vars)

    plt.show()
