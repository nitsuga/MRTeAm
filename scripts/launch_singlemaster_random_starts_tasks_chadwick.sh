#!/bin/sh
#$ -cwd
#$ -j y
#$ -pe smp 16
#$ -l exclusive
#$ -t 1-30
#$ -V
#$ -M eric.schneider@liverpool.ac.uk
#$ -m abe

source /home/esch/.bashrc
export ROS_OS_OVERRIDE=rhel

RUN_COUNT=20
SCRIPT_DIR=~/GIT/mrta/scripts

for run in `seq 1 ${RUN_COUNT}`
do
    # Generate a set of random tasks
    TASK_FILE="$($SCRIPT_DIR/random_poses.py tasks `rospack find mrta`/config/maps/smartlab_ugv_arena_v2.png --num_poses 16)"

    # Generate a set of random start poses
    ${SCRIPT_DIR}/random_poses.py starts `rospack find mrta`/config/maps/smartlab_ugv_arena_v2.png

    for mechanism in SSI SEL #PSI OSI
    do
	    ${SCRIPT_DIR}/launch_experiment_singlemaster.py -ng -rs ${mechanism} smartlab random ${TASK_FILE}
    done # end "mechanism"
done # end "run"
