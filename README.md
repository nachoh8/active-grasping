# Active Grasping

## Requirements

* [Simox](https://gitlab.com/Simox/simox)
* [BayesOpt](https://github.com/rmcantin/bayesopt)
* Python 3
* C++ 20
* SWIG (to create Grasp lib python interface)

## Compile

    >> mkdir build && cd build
    >> cmake ..
    >> make -j4
    >> sudo make install

## Execution ##

Active Grasping Optimization:

    >> python3 main_active_grasping.py   -fgopt <active_grasp_params_file>
                            -fgrasp <grasp_planner_params_file>
                            [-fbopt <bayesopt_params_file>]
                            [-flog <log_file>]

Optimization evaluation:

    >> python3 evaluation.py -flog <log_file>

Grasp visualization:

    >> ./build/bin/grasp_visualization <grasp_planner_params_file>
