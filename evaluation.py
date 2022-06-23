import argparse
import os
import glob
import numpy as np
import matplotlib.pyplot as plt

from active_grasping.datalog import DataLog

def compute_max_until_iteration(outcomes: np.ndarray) -> np.ndarray:
    res = np.array([outcomes[0]])

    for i in range(1, outcomes.shape[0]):
        v = outcomes[i] if outcomes[i] > res[i-1] else res[i-1]
        res = np.append(res, v)
    return res

def outcome_iterations(outcomes: np.ndarray, best_acum=False, errors: np.ndarray = None, names: "list[str]" = None):
    
    iterations = range(1, outcomes.shape[1]+1)
    plt.figure()
    for i, outs in zip(range(outcomes.shape[0]), outcomes):
        if best_acum:
            title = 'Best outcome until iteration x'
            Y = compute_max_until_iteration(outs)
        else:
            Y = outs
            title = 'Value of best selected sample'

        """if errors is not None:
            plt.errorbar(iterations, Y, yerr=errors[i], fmt='o', label=names[i], alpha=0.7)
        else:
            plt.plot(iterations, Y, 'o-')"""

        plt.plot(iterations, Y, label=names[i])
        if errors is not None:
            plt.fill_between(iterations, Y - errors[i], Y + errors[i], alpha=0.3)
    
    if names:
        plt.legend()
    plt.xlabel('Iteration')
    plt.ylabel('Outcome')
    plt.title(title)

def outcome_vars(queries: np.ndarray, outcomes: np.ndarray, var_labels: "list[str]", plot2D=False, name: str = ""):
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
        elif n_vars == 3:
            sc_plot = ax.scatter(queries[:, 0], queries[:, 1], queries[:, 2], c=outcomes, alpha=0.5)
            ax.set_zlabel(var_labels[2])
        elif n_vars == 6:
            vars_1 = queries[:,0].reshape(-1,2)
            vars_2 = queries[:,1].reshape(-1,2)
            vars_3 = queries[:,2].reshape(-1,2)

            sc_plot_ = ax.scatter(vars_1[:,0], vars_2[:,0], vars_3[:,0], c=outcomes, alpha=0.5)
            plt.colorbar(sc_plot_, label="outcome", orientation="vertical")
            plt.title("Distribution of the outcome " + name)

            fig2 = plt.figure()
            ax_ = fig2.add_subplot(projection='3d')
            sc_plot = ax_.scatter(vars_1[:,1], vars_2[:,1], vars_3[:,1], c=outcomes, alpha=0.5)
            ax_.set_zlabel(var_labels[5])
            ax_.set_xlabel(var_labels[3])
            ax_.set_ylabel(var_labels[4])

            ax.set_zlabel(var_labels[2])

        ax.set_xlabel(var_labels[0])
        ax.set_ylabel(var_labels[1])

    plt.colorbar(sc_plot, label="outcome", orientation="vertical")
    
    plt.title("Distribution of the outcome " + name)

def plot_best(grasps: np.ndarray, outcomes: np.ndarray, var_labels: "list[str]", plot_text = False, name: str = ""):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    for i, gs in zip(range(grasps.shape[0]), grasps):
        ax.scatter(gs[:, 0], gs[:, 1], gs[:, 2], marker='o', alpha=1.0)
        if plot_text:
            for j in range(gs.shape[0]):
                # print(gs[j], gs[j].shape)
                # print(outcomes[i], outcomes[i][j].shape)
                ax.text(gs[j, 0], gs[j, 1], gs[j, 2], str(round(outcomes[i][j], 3)))

    ax.set_xlabel(var_labels[0])
    ax.set_ylabel(var_labels[1])
    ax.set_zlabel(var_labels[2])
    
    
    plt.title("Best grasps " + name)


def get_values(file_path: str) -> "tuple[str, list[str], np.ndarray, np.ndarray, tuple[np.ndarray, np.ndarray]]":
    """
    input: log file
    output: optimizer name, active variables, grasps, outcomes, best grasps
    """

    logger = DataLog(log_file=file_path)
    logger.load_json()

    act_vars = logger.get_active_vars()
    queries, outcomes = logger.get_grasps()
    best = logger.get_best_grasps()
    metrics = logger.get_metrics()

    grasps = np.array(queries)
    res = np.array(outcomes).reshape(-1)
    print('RES: ', res.shape)

    return logger.get_optimizer_name(), act_vars, grasps, res, (np.array(best[0]), np.array(best[1]).reshape(-1))

def get_folder_values(folder_path: str) -> "tuple[str, list[str], np.ndarray, np.ndarray, np.ndarray, np.ndarray]":
    """
    input: log file
    output: optimizer name, active variables, all grasps, all outcomes, best grasps, best outcomes
    """

    logs = glob.glob(folder_path + "/*.json")
    print("Num. log files: " + str(len(logs)))

    print("Adding", logs[0])
    optimizer_name, act_vars, grasps, outcomes, best = get_values(logs[0])
    all_grasps = [grasps]
    all_outcomes = [outcomes]
    all_best_grasps = [best[0]]
    all_best_outcomes = [best[1]]
    logs.pop(0)

    for file in logs:
        print("Adding", file)
        _, _, grasps, outcomes, best = get_values(file)
        all_grasps.append(grasps)
        all_outcomes.append(outcomes)
        all_best_grasps.append(best[0])
        all_best_outcomes.append(best[1])
    
    all_grasps = np.array(all_grasps)
    all_outcomes = np.array(all_outcomes)
    all_best_grasps = np.array(all_best_grasps)
    all_best_outcomes = np.array(all_best_outcomes)

    return optimizer_name, act_vars, all_grasps, all_outcomes, all_best_grasps, all_best_outcomes


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Active Grasping with bayesian optimization')
    parser.add_argument("-flogs", nargs='+', help="log files/folders", metavar='(<log_file> | <folder>)+', required=True)

    args = parser.parse_args()
    flogs = args.flogs

    names = []
    mean_max_outcomes = []
    std_dev_max_outcomes = []

    for flog in flogs:
        if os.path.isdir(flog):  
            print("Loading results from " + flog + " folder")
            optimizer_name, act_vars, all_grasps, all_outcomes, all_best_grasps, all_best_outcomes = get_folder_values(flog)

            print("Optimizer: " + optimizer_name)
            print("Active variables: " + str(act_vars))
            print("Num. total grasps: " + str(all_grasps.shape[1]))

            max_outcomes = np.array([compute_max_until_iteration(outs) for outs in all_outcomes])
            mean_max_outcomes.append(np.mean(max_outcomes, axis=0))
            std_dev_max_outcomes.append(np.std(max_outcomes, axis=0))

            all_grasps = all_grasps.reshape(-1,3)
            all_outcomes = all_outcomes.reshape(-1,)

        else:  
            print("Loading results from " + flog + " file")
            optimizer_name, act_vars, grasps, outcomes, best = get_values(flog)

            print("Optimizer: " + optimizer_name)
            print("Active variables: " + str(act_vars))
            print("Num. grasps: " + str(grasps.shape[0]))

            all_grasps = grasps
            all_outcomes = outcomes

            all_best_grasps = np.array([best[0]])
            all_best_outcomes = np.array([best[1]])

            max_outcomes = compute_max_until_iteration(outcomes)
            mean_max_outcomes.append(max_outcomes)
            std_dev_max_outcomes.append(np.zeros((max_outcomes.shape)))
        
        names.append(optimizer_name)

        outcome_vars(all_grasps, all_outcomes, plot2D=False, var_labels=act_vars, name=optimizer_name)

        plot_best(all_best_grasps, all_best_outcomes, act_vars, name=optimizer_name, plot_text=True)
    
    outcome_iterations(np.array(mean_max_outcomes), errors=np.array(std_dev_max_outcomes), names=names)

    plt.show()
