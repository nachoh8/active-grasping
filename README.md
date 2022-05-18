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

You can choose Dev/Production token by setting the param "mode" to "dev"/"prod" in the sigopt params file.

## Tests

Tests to check that everything is working

Test c++ lib:

    >> cd build && ./bin/test_libs

Test python lib:

    >> python3 test_grasp.py
    >> python3 test_active_grasp.py
    >> python3 test_gramacy_sigop.py

## Execution

### Active Grasping Optimization

    >> python3 main_active_grasping.py
                            -fgopt <active_grasp_params_file>
                            -fgrasp <executor> <params_file>
                            -fopt <optimizer> <params_file>
                            [-flog <log_file>]

* <executor\>: {0: TestGramacyExecutor, 1: GraspPlanner, 2: GraspPlannerIK}
* <optimizer\>: {0: BayesOpt, 1: SigOpt}

### Optimization evaluation

    >> python3 evaluation.py -flog <log_file>

### Grasp visualization

#### Grasp EEF

    >> ./build/bin/grasp_visualization <params_file> [<log_file>]

* <log_file>: log file to load grasps from an experiment result

#### Grasp IK BiRRT

    >> ./build/bin/grasp_ik_visualization <params_file>

