#!/bin/bash

### PARAMS

OPT_EXECUTOR=1
GRASP_EXECUTOR=2
NUM_RUNS=10

RES_LOG_PREFIX="res_xyz"

if [ $GRASP_EXECUTOR -eq 1 ]; then
    FGRASP="config/grasp/params/grasp_params.json"
    
    FBOPT="config/bayesopt/bopt_params_default.json"
    FGOPT="config/grasp/gopt/gopt_xyz.json"

    FSOPT_OPT="config/grasp/sigopt/opt.json"
    FSOPT_MS="config/grasp/sigopt/multisol.json"
    FSOPT=$FSOPT_MS

    RES_FOLDER="logs/grasp/sigopt/xyz/ms"
elif [ $GRASP_EXECUTOR -eq 2 ]; then
    FGRASP="config/graspIK/params/grasp_params.json"

    FBOPT="config/bayesopt/bopt_params_default.json"
    FGOPT="config/graspIK/gopt/gopt_xyz.json"

    FSOPT_OPT="config/graspIK/sigopt/opt.json"
    FSOPT_MS="config/graspIK/sigopt/multisol.json"
    FSOPT=$FSOPT_MS

    RES_FOLDER_OPT="logs/graspIK/sigopt/xyz/opt"
    RES_FOLDER_MS="logs/graspIK/sigopt/xyz/ms"
    RES_FOLDER=$RES_FOLDER_MS
else
    echo "Error: Grasp executor must be 1: GraspPlanner, 2: GraspPlannerIK"
    exit 1
fi

if [ $OPT_EXECUTOR -eq 0 ]; then
    echo "Using: Bayesopt"
    echo "Bopt Params: $FBOPT"
    echo "Experiment Params: $FGOPT"

    PARAMS="-fbopt $FBOPT $FGOPT"
elif [ $OPT_EXECUTOR -eq 1 ]; then
    echo "Using: SigOpt"
    echo "SigOpt Params: $FSOPT"

    PARAMS="-fsopt $FSOPT"
else
    echo "Error: mode must be 0: Bayesopt, 1: Sigopt"
    exit 1
fi

echo "Res folder: $RES_FOLDER"
echo "Grasp Executor: ${GRASP_EXECUTOR}"
echo "Grasp Executor Params: ${FGRASP}"

### Execution
mkdir -p $RES_FOLDER
FLOG="$RES_FOLDER/$RES_LOG_PREFIX"

N_ERR=0
for (( i=3; i<=$NUM_RUNS; ))
do
    echo "-------------------------------------"
    log="${FLOG}_$i.json"
    echo "Execution $i/$NUM_RUNS -> $log"
    python3 main_active_grasping.py -fgrasp $GRASP_EXECUTOR $FGRASP $PARAMS -flog $log
    if [ $? -eq 0 ]; then
        i=$(( $i + 1 ))
        N_ERR=0
    else
        echo "Error: error in execution, trying again"
        N_ERR=$(( $N_ERR + 1 ))
        if [ $N_ERR -eq 3 ]; then
            echo "END EEXECUTIONS: execution $i fails 3 times"
            exit 1
        fi
    fi

done
