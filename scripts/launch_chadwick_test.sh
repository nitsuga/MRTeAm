#!/bin/sh
#$ -cwd
#$ -j y
#$ -pe smp 16
#$ -l exclusive
#$ -t 1-6
#$ -V

source /home/esch/.bashrc

export ROS_OS_OVERRIDE=rhel

RUN_COUNT=5
SCRIPT_DIR=~/GIT/mrta/scripts

for map in smartlab
do
    for scenario_id in three-corners.yaml
    do
	for mechanism in RR
	do
	    for start_config in clustered distributed
	    do
		for run in `seq 1 ${RUN_COUNT}`
		do
		    $SCRIPT_DIR/launch_experiment.py -ng ${mechanism} ${map} ${start_config} ${scenario_id}

		done # end "run"

	    done # end "start_config"

	done # end "mechanism"

    done # end "scenario_id"

done # end "map"
