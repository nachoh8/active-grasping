import argparse
import os
import glob
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from active_grasping.datalog import DataLog


#matplotlib.rcParams['pdf.fonttype'] = 42
#matplotlib.rcParams['ps.fonttype'] = 42

font = {'family' : 'monospace',
        'size'   : 40}

matplotlib.rc('font', **font)

COLORS=['b', 'r', 'g', '#ff7700']

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

        plt.plot(iterations, Y, label=names[i], color = COLORS[i])
        if errors is not None:
            plt.fill_between(iterations, Y - errors[i], Y + errors[i], alpha=0.3, color=COLORS[i])
    
    if names:
        plt.legend()
    plt.xlabel('Iteration')
    plt.ylabel('Outcome')
    plt.title(title, fontsize=40)


def plot_best(grasps: np.ndarray, outcomes: np.ndarray, rhos: np.ndarray, var_labels: "list[str]", plot_text = False, name: str = ""):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    rhos_ = rhos.reshape(1,rhos.shape[0])[0]
    if sum(rhos_) != 0.:
        for i, gs in zip(range(grasps.shape[0]), grasps):
            ax.scatter(gs[:, 0], gs[:, 1], rhos[i], marker='o', alpha=1.0)
            if plot_text:
                for j in range(gs.shape[0]):
                    ax.text(gs[j, 0], gs[j, 1], rhos[i], str(round(outcomes[i][j], 3)))

        ax.set_xlabel(var_labels[0])
        ax.set_ylabel(var_labels[1])
        ax.set_zlabel('rho') 
    else:
        for i, gs in zip(range(grasps.shape[0]), grasps):
            ax.scatter(gs[:, 0], gs[:, 1], gs[:, 2], marker='o', alpha=1.0)
            if plot_text:
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
    #rhos = logger.get_rho()
    #rolls, pitchs, yaws = logger.get_rpy()

    grasps = np.array(queries)
    res = np.array(outcomes).reshape(-1)
    #rhos = np.array(rhos).reshape(-1)

    best_outcomes = np.array(best[1]).reshape(-1)
    # best_queries = np.array(best[0]).reshape(-1)

    n_grasps = grasps.shape[0]
    #sum_rhos = sum(rhos)
    x = []
    y = []
    z = []
    roll = []
    pitch = []
    yaw = []
    """"
    for i in range(n_grasps):

        s1 = grasps[i]

        if sum_rhos != 0. and grasps.shape[1] == 3 :
            s1[2] = rhos[i]
        elif sum_rhos != 0.:
            s1 = np.append(s1, rhos[i])
        
        if np.any(res[i] == best_outcomes):

            x.append(s1[2]*np.sin(s1[0])*np.cos(s1[1]))
            y.append(s1[2]*np.sin(s1[0])*np.sin(s1[1]))
            z.append(s1[2]*np.cos(s1[0]))

            roll.append(rolls[i])
            pitch.append(pitchs[i])
            yaw.append(yaws[i])
    """
    print("SHAPE: ", res.shape)
    return logger.get_optimizer_name(), act_vars,x,y,z,roll,pitch,yaw, res

def get_folder_values(folder_path: str) -> "tuple[str, list[str], np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]":
    """
    input: log file
    output: optimizer name, active variables, all grasps, all outcomes, best grasps, best outcomes
    """

    logs = glob.glob(folder_path + "/*.json")
    print("Num. log files: " + str(len(logs)))

    print("Adding", logs[0])
    optimizer_name, act_vars, x, y, z, roll, pitch , yaw, outcomes = get_values(logs[0])
    all_x = [x]
    all_y = [y]
    all_z = [z]
    all_roll = [roll]
    all_pitch = [pitch]
    all_yaw = [yaw]
    all_outcomes = [outcomes]

    for file in logs[1:]:
        
        print("Adding", file)
        _, _, x, y, z, roll, pitch , yaw, outcomes = get_values(file)
        all_x.append(x)
        all_y.append(y)
        all_z.append(z)
        all_roll.append(roll)
        all_pitch.append(pitch)
        all_yaw.append(yaw)
        all_outcomes.append(outcomes)

    all_x = np.array(x)
    all_y = np.array(y)
    all_z = np.array(z)
    all_roll = np.array(roll)
    all_pitch = np.array(pitch)
    all_yaw = np.array(yaw)
    all_outcomes = np.array(all_outcomes)


    return optimizer_name, act_vars, all_x, all_y, all_z, all_roll, all_pitch, all_yaw, all_outcomes


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Active Grasping with bayesian optimization')
    parser.add_argument("-flogs", nargs='+', help="log files/folders", metavar='(<log_file> | <folder>)+', required=True)

    args = parser.parse_args()
    flogs = args.flogs

    names = ["GP", "GPS", "GPSi", "GPSo"]
    #names = ["SO-ms", "SO-constraint"]
    mean_max_outcomes = []
    std_dev_max_outcomes = []
    best_grasps = [[], []]

    for flog in flogs:
        print("-------------------------------------------------")
        n_logs = 0
        var_queries = []
        if os.path.isdir(flog):  
            print("Loading results from " + flog + " folder")
            optimizer_name, act_vars, all_x, all_y, all_z, all_roll, all_pitch, all_yaw, all_outcomes = get_folder_values(flog)
            n_logs = all_outcomes.shape[0]
            max_outcomes = np.array([compute_max_until_iteration(outs) for outs in all_outcomes])
            
            mean_max_outcomes.append(np.mean(max_outcomes, axis=0))

            std_dev_max_outcomes.append(np.std(max_outcomes, axis=0))

            all_outcomes = all_outcomes.reshape(-1,)

        else:  
            print("Loading results from " + flog + " file")
            n_logs = 1
            optimizer_name, act_vars, x,y,z,roll,pitch,yaw, res = get_values(flog)

        
    outcome_iterations(np.array(mean_max_outcomes), errors=np.array(std_dev_max_outcomes), names=names)


    plt.show()
