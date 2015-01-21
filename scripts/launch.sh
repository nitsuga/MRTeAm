#!/bin/sh

RUN_COUNT=5

for mechanism in OSI SSI PSI RR
do
    for map in brooklyn # smartlab
    do
	for start_config in distributed # clustered
	do
	    for task_file in brooklyn_tasks_A.txt brooklyn_tasks_C.txt
	    do
		for run in `seq 1 ${RUN_COUNT}`
		do

		    ./launch_experiment.py ${mechanism} ${map} ${start_config} ${task_file}

		done # end "run"

	    done # end "task_file"

	done # end "start_config"

    done # end "map"

done # end "mechanism"
