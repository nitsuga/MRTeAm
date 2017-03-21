#!/bin/sh
#$ -cwd
#$ -j y
#$ -pe smp 16
#$ -l exclusive
#$ -t 1-6
#$ -V

#source /home/esch/.bashrc

#export ROS_OS_OVERRIDE=rhel

RUN_COUNT=50
SCRIPT_DIR=~/GIT/mrta/scripts


for run in `seq 1 ${RUN_COUNT}`
do
    for mechanism in SSI PSI OSI
    do
	    $SCRIPT_DIR/launch_experiment_singlemaster.py -ng -dm ${mechanism} smartlab random SR-IT-SA-scenario3-16task
	done # end "mechanism"
done # end "run"
