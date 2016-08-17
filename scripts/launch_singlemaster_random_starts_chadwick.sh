#!/bin/sh
#$ -cwd
#$ -j y
#$ -pe smp 16
#$ -l exclusive
#$ -t 1-5
#$ -V

#source /home/esch/.bashrc

#export ROS_OS_OVERRIDE=rhel

RUN_COUNT=20
SCRIPT_DIR=~/GIT/mrta/scripts


for run in `seq 1 ${RUN_COUNT}`
do
    for mechanism in SSI PSI OSI
    do
	    $SCRIPT_DIR/launch_experiment_singlemaster.py -ng ${mechanism} smartlab random SR-IT-SA-scenario3-16task.yaml
	done # end "mechanism"
done # end "run"
