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
TASK_BUFFER=10
START_CONFIG=random

CLASSIFIER=strand-restricted/smote-enn/clf_execution_phase_time_random_forest
# CLASSIFIER=strand-restricted/undersample-random/clf_execution_phase_time_random_forest

SCENARIO_ID_LIST=($(/bin/cat ${SCRIPT_DIR}/scenario_id_list.txt))
ROBOT_POSES=($(/bin/cat ${SCRIPT_DIR}/robot_start_position_list.txt))

RUN_COUNT=${#SCENARIO_ID_LIST[*]} # The length of the SCENARIO_ID_LIST array

for run in `seq 0 $((${RUN_COUNT} - 1))`
do
    # Generate a set of random tasks
#    SCENARIO_ID="$(${SCRIPT_DIR}/random_poses.py tasks `rospack find mrta`/config/maps/${MAP_IMAGE} --num_poses ${NUM_TASKS} --buffer ${TASK_BUFFER} --scale ${MAP_SCALE})"

    SCENARIO_ID=${SCENARIO_ID_LIST[${run}]}

    POSE_BASE_INDEX=$((run * 6))

    # Generate a set of random start poses
#    ${SCRIPT_DIR}/random_poses.py starts `rospack find mrta`/config/maps/${MAP_IMAGE} --buffer ${ROBOT_BUFFER} --scale ${MAP_SCALE}
    r1x=${ROBOT_POSES[${POSE_BASE_INDEX}]}
    r1y=${ROBOT_POSES[$((${POSE_BASE_INDEX}+1))]}
    r2x=${ROBOT_POSES[$((${POSE_BASE_INDEX}+2))]}
    r2y=${ROBOT_POSES[$((${POSE_BASE_INDEX}+3))]}
    r3x=${ROBOT_POSES[$((${POSE_BASE_INDEX}+4))]}
    r3y=${ROBOT_POSES[$((${POSE_BASE_INDEX}+5))]}

    ${SCRIPT_DIR}/random_poses.py manual-starts `rospack find mrta`/config/maps/${MAP_IMAGE} --num_poses 3 --poses ${r1x} ${r1y} ${r2x} ${r2y} ${r3x} ${r3y}

    for mechanism in PSI # PSI SSI # OSI
    do
	    ${SCRIPT_DIR}/launch_experiment_singlemaster.py -rs -cl ${CLASSIFIER} ${mechanism} ${MAP_NAME} ${START_CONFIG} ${SCENARIO_ID}
    done # end "mechanism"
done # end "run"
