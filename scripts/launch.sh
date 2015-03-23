#!/bin/sh

RUN_COUNT=10

#for map in brooklyn # smartlab
for map in smartlab
do
    #for task_file in brooklyn_tasks_A.txt brooklyn_tasks_C.txt brooklyn_tasks_E.txt
    for task_file in tasks_A.txt
    do
	for mechanism in OSI SSI PSI RR
	do
	    for start_config in distributed clustered
	    do
		for run in `seq 1 ${RUN_COUNT}`
		do
		    ./launch_experiment.py ${mechanism} ${map} ${start_config} ${task_file}

		done # end "run"

	    done # end "start_config"

	done # end "mechanism"

    done # end "task_file"

done # end "map"
