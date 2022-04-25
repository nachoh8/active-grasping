# Active Grasping

## Requirements

* Python 3
* C++ 20
* [Simox](https://gitlab.com/Simox/simox)
* [BayesOpt](https://github.com/rmcantin/bayesopt)
* [SigOpt - Python](https://sigopt.com/)
* SWIG (to create Grasp lib python interface)

## Compile

    >> mkdir build && cd build
    >> cmake ..
    >> make -j4
    >> sudo make install

## SigOpt setup

If you want to use active grasping with SigOpt platfform you have to add two environment variables:

* SIGOPT_DEV_TOKEN, your sigopt development token
* SIGOPT_PROD_TOKEN, your sigopt production token

Dev token is used by default, you can select production token with the argument "--prod"

## Execution

Active Grasping Optimization:

    >> python3 main_active_grasping.py   -fgopt <active_grasp_params_file>
                            -fgrasp <grasp_planner_params_file>
                            [-fbopt <bayesopt_params_file>]
                            [-flog <log_file>]

Optimization evaluation:

    >> python3 evaluation.py -flog <log_file>

Grasp visualization:

    >> ./build/bin/grasp_visualization <grasp_planner_params_file> [<log_file>]

* <log_file>: log file to load grasps from an experiment
