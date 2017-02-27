#!/bin/sh

RUN_COUNT=25
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

for run in `seq 1 ${RUN_COUNT}`
do
    # Generate a set of random tasks
    TASK_FILE="$(${SCRIPT_DIR}/random_poses.py tasks `rospack find mrta`/config/maps/${MAP_IMAGE} --num_poses ${NUM_TASKS} --buffer ${TASK_BUFFER} --scale ${MAP_SCALE})"

    # Generate a set of random start poses
    ${SCRIPT_DIR}/random_poses.py starts `rospack find mrta`/config/maps/${MAP_IMAGE} --buffer ${ROBOT_BUFFER} --scale ${MAP_SCALE}

    for mechanism in SSI PSI # SEL OSI
    do
	    ${SCRIPT_DIR}/launch_experiment_singlemaster.py -rs ${mechanism} ${MAP_NAME} ${START_CONFIG} ${TASK_FILE}
    done # end "mechanism"
done # end "run"
