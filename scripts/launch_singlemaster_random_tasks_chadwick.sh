#!/bin/sh
#$ -cwd
#$ -j y
#$ -pe smp 16
#$ -l exclusive
#$ -t 1-10
#$ -V

#source /home/esch/.bashrc

#export ROS_OS_OVERRIDE=rhel

RUN_COUNT=1
SCRIPT_DIR=~/GIT/mrta/scripts

for run in `seq 1 ${RUN_COUNT}`
do
    # Generate a set of random tasks
    TASK_FILE="$($SCRIPT_DIR/random_poses.py tasks `rospack find mrta`/config/maps/smartlab_ugv_arena_v2.png --num_poses 16)"

    for mechanism in SSI PSI #OSI
    do
	    $SCRIPT_DIR/launch_experiment_singlemaster.py -dm -rs ${mechanism} smartlab clustered ${TASK_FILE}
	done # end "mechanism"
done # end "run"
