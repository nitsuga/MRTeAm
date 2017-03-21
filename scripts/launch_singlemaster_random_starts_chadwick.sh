#!/bin/sh
#$ -cwd
#$ -j y
#$ -pe smp 16
#$ -l exclusive
#$ -t 1-10
#$ -V
#$ -M eric.schneider@liverpool.ac.uk
#$ -m abe

source /home/esch/.bashrc
export ROS_OS_OVERRIDE=rhel

RUN_COUNT=1
SCRIPT_DIR=~/GIT/mrta/scripts

for run in `seq 1 ${RUN_COUNT}`
do
    # Generate a set of random start poses
    $SCRIPT_DIR/random_poses.py starts `rospack find mrta`/config/maps/smartlab_ugv_arena_v2.png

    for mechanism in SSI PSI #OSI
    do
	    $SCRIPT_DIR/launch_experiment_singlemaster.py -dm -rs ${mechanism} smartlab random SR-IT-SA-scenario3-16task
	done # end "mechanism"
done # end "run"
