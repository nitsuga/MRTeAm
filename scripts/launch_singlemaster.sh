#!/bin/sh
#$ -cwd
#$ -j y
#$ -pe smp 16
#$ -l exclusive
#$ -t 1-6
#$ -V

#source /home/esch/.bashrc

#export ROS_OS_OVERRIDE=rhel

RUN_COUNT=15
SCRIPT_DIR=~/GIT/mrta/scripts

for map in smartlab
do
#    for task_file in MR-IT-DA-scenario1.yaml MR-CT-DA-scenario1.yaml SR-IT-DA-scenario1.yaml SR-CT-DA-scenario1.yaml
    for task_file in MR-IT-DA-scenario1.yaml MR-CT-DA-scenario1.yaml MR-IT-DA-scenario2.yaml MR-CT-DA-scenario2.yaml
    do
	for mechanism in OSI SSI PSI RR
	do
	    for start_config in clustered distributed
	    do
		for run in `seq 1 ${RUN_COUNT}`
		do
		    $SCRIPT_DIR/launch_experiment_singlemaster.py -ng ${mechanism} ${map} ${start_config} ${task_file}

		done # end "run"

	    done # end "start_config"

	done # end "mechanism"

    done # end "task_file"

done # end "map"
