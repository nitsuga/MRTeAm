#!/bin/sh
#$ -cwd
#$ -j y
#$ -pe smp 16
#$ -l exclusive
#$ -t 1-85
#$ -V
#$ -M eric.schneider@liverpool.ac.uk
#$ -m abe

source /home/esch/.bashrc
export ROS_OS_OVERRIDE=rhel

RUN_COUNT=12
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
    SCENARIO_ID="$(${SCRIPT_DIR}/random_poses.py tasks `rospack find mrta`/config/maps/${MAP_IMAGE} --num_poses ${NUM_TASKS} --buffer ${TASK_BUFFER} --scale ${MAP_SCALE} --dynamic_rate 45)"

    # Generate a set of random start poses
    ${SCRIPT_DIR}/random_poses.py starts `rospack find mrta`/config/maps/${MAP_IMAGE} --buffer ${ROBOT_BUFFER} --scale ${MAP_SCALE}

    for mechanism in SSI PSI # OSI
    do
	    ${SCRIPT_DIR}/launch_experiment_singlemaster.py -ng -rs ${mechanism} ${MAP_NAME} ${START_CONFIG} ${SCENARIO_ID}
    done # end "mechanism"
done # end "run"

