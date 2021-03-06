#!/bin/bash

# RUN_COUNT=25
SCRIPT_DIR=~/GIT/mrta/scripts

NUM_TASKS=20

# MAP_NAME=smartlab
# MAP_IMAGE=smartlab_ugv_arena_v2.png
# MAP_SCALE=1.0
# ROBOT_BUFFER=70
# TASK_BUFFER=30
# START_CONFIG=random

MAP_NAME=strand-restricted
MAP_IMAGE=map-strand-first-floor-restricted-5cm.png
MAP_SCALE=6.65
ROBOT_BUFFER=15
TASK_BUFFER=20
START_CONFIG=random

CLASSIFIER=strand-restricted/smote-enn/clf_execution_phase_time_random_forest
# CLASSIFIER=strand-restricted/undersample-random/clf_execution_phase_time_random_forest

RUN_COUNT=1

for run in `seq 0 $((${RUN_COUNT} - 1))`
do
    # Generate a set of random tasks
    SCENARIO_ID="$(${SCRIPT_DIR}/random_poses.py tasks `rospack find mrta`/config/maps/${MAP_IMAGE} --num_poses ${NUM_TASKS} --buffer ${TASK_BUFFER} --scale ${MAP_SCALE})" # --multirobot)" # --constrained --dynamic_rate 30)"

#    SCENARIO_ID=MR-IT-SA-6QPYML88-20task

    # Generate a set of random start poses
    ${SCRIPT_DIR}/random_poses.py starts `rospack find mrta`/config/maps/${MAP_IMAGE} --buffer ${ROBOT_BUFFER} --scale ${MAP_SCALE}

    for mechanism in SSI # PSI SSI # OSI
    do
	    ${SCRIPT_DIR}/launch_experiment_singlemaster.py -rs -cl ${CLASSIFIER} ${mechanism} ${MAP_NAME} ${START_CONFIG} ${SCENARIO_ID}
    done # end "mechanism"
done # end "run"
