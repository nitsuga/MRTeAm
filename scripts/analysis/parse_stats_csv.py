#!/usr/bin/env python

from collections import defaultdict
import csv
import glob
import os
import re
import rosbag
import sys

CSV_FILENAME = 'stats.csv'

field_names = [
#    'BAG_FILENAME',
    'DATETIME',
    'MAP',
    'START_CONFIG',
    'MECHANISM',
    'TASK_FILE',
#    'RUN_NUM',
#    'ATTEMPT_NUM',
    'TOTAL_RUN_TIME',
    'DELIBERATION_TIME',
    'EXECUTION_TIME',
    'TOTAL_IDLE_TIME',
    'TOTAL_DELAY_TIME',
    'TOTAL_DISTANCE',
    'TOTAL_COLLISIONS',
    'ALLOC_MSGS',
    'ALLOC_TASK_MSGS',
    'ALLOC_MSG_BYTES',
    'ROBOT1_DISTANCE',
    'ROBOT1_TRAVEL_TIME',
    'ROBOT1_IDLE_TIME',
    'ROBOT1_DELAY_TIME',
    'ROBOT2_DISTANCE',
    'ROBOT2_TRAVEL_TIME',
    'ROBOT2_IDLE_TIME',
    'ROBOT2_DELAY_TIME',
    'ROBOT3_DISTANCE',
    'ROBOT3_TRAVEL_TIME',
    'ROBOT3_IDLE_TIME',
    'ROBOT3_DELAY_TIME',
]

ROBOT_NAMES = [ 'robot_1',
                'robot_2',
                'robot_3' ]

def usage():
    print 'Usage: ' + sys.argv[0] + ': <path to .bag file(s)>'

def main(argv):

    if len(argv) < 1:
        usage()
        sys.exit(1)

    # Make one, flat list of paths to log files
    bag_paths = []

    for path_arg in argv:
        bag_paths.extend(glob.glob(path_arg))

    # Open the CSV file for writing
    csv_file = csv.writer(open(CSV_FILENAME, 'wb'))
    csv_file.writerow(field_names)

    dt_re = re.compile('(.*)\.bag')

    for i,bag_path in enumerate(bag_paths):
        print("Reading {0}".format(bag_path))

        bag = None
        try:
            bag = rosbag.Bag(bag_path)
        except:
            print("Couldn't open {0} for reading!".format(bag_path))
            continue
        
        row_fields = []
        bag_filename = os.path.basename(bag_path)
        #row_fields.append(bag_filename)


        (map, start_config, mechanism, task_file, remainder) = bag_filename.split('__')
        
        dt_match = dt_re.search(remainder)

        # Date/time
        row_fields.append(dt_match.group(1))

        row_fields.extend([map, start_config, mechanism, task_file])


        run_msgs = defaultdict(list)

        for topic,msg,msg_time in bag.read_messages():
            run_msgs[topic].append(msg)

        exp_msgs = {}
        for msg in run_msgs['/experiment']:
            exp_msgs[msg.event] = msg

        # Total run time
        total_diff = (exp_msgs['END_EXPERIMENT'].header.stamp - 
                      exp_msgs['BEGIN_EXPERIMENT'].header.stamp)
        total_run_time = (total_diff.secs + total_diff.nsecs/1000000000.)
        row_fields.append(total_run_time)

        # Deliberation time
        delib_diff = (exp_msgs['END_ALLOCATION'].header.stamp - 
                      exp_msgs['BEGIN_ALLOCATION'].header.stamp)
        delib_time = (delib_diff.secs + delib_diff.nsecs/1000000000.)
        row_fields.append(delib_time)

        # Execution time
        exec_diff = (exp_msgs['END_EXECUTION'].header.stamp - 
                      exp_msgs['BEGIN_EXECUTION'].header.stamp)
        exec_time = (exec_diff.secs + exec_diff.nsecs/1000000000.)
        row_fields.append(exec_time)

        for robot in ROBOT_NAMES:
            pass


        csv_file.writerow(row_fields)

if __name__ == '__main__':
    main(sys.argv[1:])
