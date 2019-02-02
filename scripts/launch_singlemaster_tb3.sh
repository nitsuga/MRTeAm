#!/bin/sh

RUN_COUNT=30
#SCRIPT_DIR=/GIT/mrta/scripts
SCRIPT_DIR=/home/k1759936/ros/mrteam_ws/src/mrta/scripts

for map in seb15-mr
do
    for scenario_id in MR-IT-SA-SEB15-scenario1-simulation.yaml MR-IT-SA-SEB15-scenario2-simulation.yaml MR-CT-SA-SEB15-scenario1-simulation.yaml MR-CT-SA-SEB15-scenario2-simulation.yaml
 # SR-IT-SA-SEB15-scenario1.yaml SR-IT-SA-SEB15-scenario2.yaml SR-CT-SA-SEB15-scenario1.yaml SR-CT-SA-SEB15-scenario2.yaml
    do
	for mechanism in OSI SSI PSI RR
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
