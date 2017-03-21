#!/bin/sh
#$ -cwd
#$ -j y
#$ -pe smp 16
#$ -l exclusive
#$ -t 1-6
#$ -V

#source /home/esch/.bashrc

#export ROS_OS_OVERRIDE=rhel

RUN_COUNT=20
SCRIPT_DIR=~/GIT/mrta/scripts

for map in smartlab
do
#    for scenario_id in MR-IT-DA-scenario1 MR-CT-DA-scenario1 SR-IT-DA-scenario1 SR-CT-DA-scenario1
    for scenario_id in SR-IT-SA-scenario3-16task
    do
	for mechanism in SSI PSI
	do
	    for start_config in clustered distributed
	    do
		for run in `seq 1 ${RUN_COUNT}`
		do
		    $SCRIPT_DIR/launch_experiment_singlemaster.py -ng ${mechanism} ${map} ${start_config} ${scenario_id}

		done # end "run"

	    done # end "start_config"

	done # end "mechanism"

    done # end "scenario_id"

done # end "map"
