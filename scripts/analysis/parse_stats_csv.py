#!/usr/bin/env python

import argparse
from collections import defaultdict
import csv
import glob
import math
import os
import pprint
import re
import rosbag
import sys

import mrta
import mrta.msg

DEFAULT_CSV_FILENAME = 'stats.csv'

field_names = [
    'BAG_FILENAME',
    'DATETIME',
    'MAP',
    'START_CONFIG',
    'MECHANISM',
    'TASK_FILE',
#    'RUN_NUM',
#    'ATTEMPT_NUM',
    'TOTAL_RUN_TIME',
    'DELIBERATION_TIME',
    'EXECUTION_PHASE_TIME',
    'TOTAL_MOVEMENT_TIME',
    'TOTAL_EXECUTION_TIME',
    'TOTAL_WAITING_TIME',
    'TOTAL_IDLE_TIME',
    'TOTAL_DELAY_TIME',
    'TOTAL_DISTANCE',
    'TOTAL_COLLISIONS',
    'NUM_ANNOUNCE_MSGS',
    'NUM_ANNOUNCE_TASKS',
    'NUM_BID_MSGS',
    'ALLOC_MSG_BYTES',
    'ROBOT1_DISTANCE',
    'ROBOT1_MOVEMENT_TIME',
    'ROBOT1_WAITING_TIME',
    'ROBOT1_IDLE_TIME',
    'ROBOT1_DELAY_TIME',
    'ROBOT2_DISTANCE',
    'ROBOT2_MOVEMENT_TIME',
    'ROBOT2_WAITING_TIME',
    'ROBOT2_IDLE_TIME',
    'ROBOT2_DELAY_TIME',
    'ROBOT3_DISTANCE',
    'ROBOT3_MOVEMENT_TIME',
    'ROBOT3_WAITING_TIME',
    'ROBOT3_IDLE_TIME',
    'ROBOT3_DELAY_TIME',
]

ROBOT_NAMES = [ 'robot_1',
                'robot_2',
                'robot_3' ]

pp = pprint.PrettyPrinter(indent=4)

class Robot(object):
    def __init__(self):
        self.distance = 0.0
        self.movement_time = 0.0
        self.idle_time = 0.0
        self.delay_time = 0.0

        self.collisions = 0

        self.exec_phase_begin_stamp = None
        self.exec_phase_end_stamp = None

        # Times at which this robot paused during execution of tasks. Keys are task ids
        # and values are timestamps
        self.pause_times = defaultdict(int)

def count_interval_times(start_event, end_event, messages):
    """
    Count the time in all [start_event,end_event] intervals and return the sum.
    """
    interval_times = 0.0

    last_start_stamp = None

    # The name of a message's 'event' field depends on its class.
    attr_name = None
    sample_msg = messages[0]
#    if isinstance(sample_msg, mrta.msg.ExperimentEvent):
    if sample_msg.__class__.__name__ == '_mrta__ExperimentEvent':
        attr_name = 'event'
#    elif isinstance(sample_msg, mrta.msg.TaskStatus):
    elif sample_msg.__class__.__name__ == '_mrta__TaskStatus':
        attr_name = 'status'
    else:
        # Should actually raise an error here
        print('Unrecognized message class: {0}'.format(sample_msg.__class__.__name__))
        return interval_times

    for message in messages:
        event = message.__getattribute__(attr_name)
        if  event == start_event:
            last_start_stamp = message.header.stamp
            continue
        elif event == end_event:
            if not last_start_stamp:
                print("count_interval_time(): {0} not preceded by a {1}!".format(end_event, start_event))
                continue
            else:
                interval_time = message.header.stamp - last_start_stamp
                interval_secs = interval_time.secs + (interval_time.nsecs/1000000000.)
                print("{0}--{1}=={2}".format(start_event, end_event, interval_secs))
                interval_times += interval_secs
                last_start_stamp = None

    print("total: {0}".format(interval_times))
    return interval_times

def parse_stats(bag_paths, output):

    # Open the CSV file for writing
    csv_file = csv.writer(open(output, 'wb'))
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

        row_fields = {}
        bag_filename = os.path.basename(bag_path)
        row_fields['BAG_FILENAME'] = bag_filename

        (map, start_config, mechanism, task_file, remainder) = bag_filename.split('__')

        # Date/time
        dt_match = dt_re.search(remainder)
        row_fields['DATETIME'] = dt_match.group(1)

        # 'START_CONFIG', 'MECHANSIM', 'TASK_FILE'
#        row_fields.extend([map, start_config, mechanism, task_file])
        row_fields['MAP'] = map
        row_fields['START_CONFIG'] = start_config
        row_fields['MECHANISM'] = mechanism
        row_fields['TASK_FILE'] = task_file

        run_msgs = defaultdict(list)

        for topic,msg,msg_time in bag.read_messages():
            run_msgs[topic].append(msg)

        exp_msgs = []
        experiment_finished = False
        for msg in run_msgs['/experiment']:
            if msg.event == mrta.msg.ExperimentEvent.END_EXPERIMENT:
                experiment_finished = True
            exp_msgs.append(msg)

        #if 'END_EXPERIMENT' not in exp_msgs:
        if not experiment_finished:
            print("Experiment timed out! Skipping...")
            continue

        print('TOTAL_RUN_TIME:')
        total_run_time = count_interval_times(mrta.msg.ExperimentEvent.BEGIN_EXPERIMENT,
                                              mrta.msg.ExperimentEvent.END_EXPERIMENT,
                                              exp_msgs)
        row_fields['TOTAL_RUN_TIME'] = total_run_time # 'TOTAL_RUN_TIME'

        print('DELIBERATION_TIME:')
        delib_time = count_interval_times(mrta.msg.ExperimentEvent.BEGIN_ALLOCATION,
                                          mrta.msg.ExperimentEvent.END_ALLOCATION,
                                          exp_msgs)
        row_fields['DELIBERATION_TIME'] = delib_time # 'DELIBERATION_TIME'

        print('EXECUTION_PHASE_TIME:')
        exec_phase_time = count_interval_times(mrta.msg.ExperimentEvent.BEGIN_EXECUTION,
                                               mrta.msg.ExperimentEvent.END_EXECUTION,
                                               exp_msgs)
        row_fields['EXECUTION_PHASE_TIME'] = exec_phase_time # 'EXECUTION_PHASE_TIME'


        # The timestamp of the last 'END_EXECUTION' message
        exp_end_execution_stamp = None
        for msg in exp_msgs:
            if msg.event == mrta.msg.ExperimentEvent.END_EXECUTION:
                exp_end_execution_stamp = msg.header.stamp

        total_movement_time = 0.0
        total_execution_time = 0.0
        total_waiting_time = 0.0
        total_idle_time = 0.0
        total_delay_time = 0.0
        total_distance = 0.0
        total_collisions = 0

        robots = {}

        # per-robot stats
        for r_name in ROBOT_NAMES:
            robot = Robot()
            robots[r_name] = robot

            # Distance travelled
            robot.distance = 0.0
            last_pose = None
            for amcl_msg in run_msgs['/{0}/amcl_pose'.format(r_name)]:

                amcl_pose = amcl_msg.pose.pose
                if amcl_pose.position.x == 0 and amcl_pose.position.y == 0:
                    continue

                #print("last_pose: {0}".format(last_pose))
                #print("amcl_pose: {0}".format(last_pose))

                if last_pose is None:
                    last_pose = amcl_pose
                    continue

                pos_delta = math.hypot(amcl_pose.position.x - last_pose.position.x,
                                       amcl_pose.position.y - last_pose.position.y)
#                print "{0} pos_delta: from ({1},{2}) to ({3},{4}) is {5}".format(r_name,
#                                                                                 amcl_pose.position.x,
#                                                                                 amcl_pose.position.y,
#                                                                                 last_pose.position.x,
#                                                                                 last_pose.position.y,
#                                                                                 pos_delta)
                robot.distance += pos_delta
                last_pose = amcl_pose

#            print "{0} distance: {1}".format(r_name, robot.distance)

            r_status_msgs = [m for m in run_msgs['/tasks/status'] if m.robot_id == r_name]

            print('{0} movement time:'.format(r_name))
            # Movement time
            robot.movement_time = count_interval_times(mrta.msg.TaskStatus.MOVING,
                                                       mrta.msg.TaskStatus.ARRIVED,
                                                       r_status_msgs)

            print('{0} execution time:'.format(r_name))
            # Execution time
            robot.execution_time = count_interval_times(mrta.msg.TaskStatus.BEGIN,
                                                        mrta.msg.TaskStatus.SUCCESS,
                                                        r_status_msgs)

            print('{0} waiting time:'.format(r_name))
            # Waiting time
            robot.waiting_time = count_interval_times(mrta.msg.TaskStatus.ARRIVED,
                                                      mrta.msg.TaskStatus.BEGIN,
                                                      r_status_msgs)

            print('{0} delay time:'.format(r_name))
            # Delay time
            robot.delay_time = count_interval_times(mrta.msg.TaskStatus.PAUSE,
                                                    mrta.msg.TaskStatus.RESUME,
                                                    r_status_msgs)

            for status_msg in r_status_msgs:

                if status_msg.status == mrta.msg.TaskStatus.MOVING and robot.exec_phase_begin_stamp is None:
                    robot.exec_phase_begin_stamp = status_msg.header.stamp

                #if status_msg.status == 'ALL_TASKS_COMPLETE':
                if status_msg.status == mrta.msg.TaskStatus.SUCCESS or status_msg == mrta.msg.TaskStatus.FAILURE:
                    robot.exec_phase_end_stamp = status_msg.header.stamp

                if status_msg.status == mrta.msg.TaskStatus.PAUSE:
                    robot.collisions += 1

            if not robot.exec_phase_begin_stamp or not robot.exec_phase_end_stamp:
                print("Can not determine beginning or end time of {0}'s exec phase!".format(r_name))
                continue

            #idle_time_diff = exp_msgs['END_EXECUTION'].header.stamp - robot.travel_end_time
            idle_time_diff = exp_end_execution_stamp - robot.exec_phase_end_stamp
            robot.idle_time = (idle_time_diff.secs + idle_time_diff.nsecs/1000000000.)

            # Since messages may arrive out of order it's possible for the idle time
            # of the last robot to be negative. Make the minimum time 0 here.
            if robot.idle_time < 0:
                robot.idle_time = 0

            print("{0} collisions: {1}".format(r_name, robot.collisions))

            total_movement_time += robot.movement_time
            total_execution_time += robot.execution_time
            total_waiting_time += robot.waiting_time
            total_idle_time += robot.idle_time
            total_delay_time += robot.delay_time
            total_distance += robot.distance
            total_collisions += robot.collisions

        print("total_collisions: {0}".format(total_collisions))

        row_fields['TOTAL_MOVEMENT_TIME'] = total_movement_time   # 'TOTAL_MOVEMENT_TIME'
        row_fields['TOTAL_EXECUTION_TIME'] = total_execution_time # 'TOTAL_EXECUTION_TIME'
        row_fields['TOTAL_WAITING_TIME'] = total_waiting_time     # 'TOTAL_WAITING_TIME'
        row_fields['TOTAL_IDLE_TIME'] = total_idle_time           # 'TOTAL_IDLE_TIME'
        row_fields['TOTAL_DELAY_TIME'] = total_delay_time         # 'TOTAL_DELAY_TIME'
        row_fields['TOTAL_DISTANCE'] = total_distance             # 'TOTAL_DISTANCE'
        row_fields['TOTAL_COLLISIONS'] = total_collisions         # 'TOTAL_COLLISIONS'

        # Deliberation metrics
        announce_msg_bytes = bid_msg_bytes = award_msg_bytes = 0

        # Number of announcement messages
        num_announce_msgs = len(run_msgs['/tasks/announce'])
        row_fields['NUM_ANNOUNCE_MSGS'] = num_announce_msgs       # 'NUM_ANNOUNCE_MSGS'

        num_ann_tasks = 0
        for ann_msg in run_msgs['/tasks/announce']:
            num_ann_tasks += len(ann_msg.tasks)
            announce_msg_bytes = sys.getsizeof(ann_msg)

        row_fields['NUM_ANNOUNCE_TASKS'] = num_ann_tasks          # 'NUM_ANNOUNCE_TASKS'

        num_bid_msgs = 0
        for bid_msg in run_msgs['/tasks/bid']:
            num_bid_msgs += 1
            bid_msg_bytes = sys.getsizeof(bid_msg)

        row_fields['NUM_BID_MSGS'] = num_bid_msgs                 # 'NUM_BID_MSGS'

        for award_msg in run_msgs['/tasks/award']:
            award_msg_bytes = sys.getsizeof(award_msg)

        alloc_msg_bytes = announce_msg_bytes + bid_msg_bytes + award_msg_bytes

        row_fields['ALLOC_MSG_BYTES'] = alloc_msg_bytes          # 'ALLOC_MSGS_BYTES'

#        for r_name in ROBOT_NAMES:
#            robot = robots[r_name]

        for i in range(1,4):
            r_name = 'robot_{0}'.format(i)
            robot = robots[r_name]

            row_fields['ROBOT{0}_DISTANCE'.format(i)] = robot.distance           # 'ROBOT<n>_DISTANCE'
            row_fields['ROBOT{0}_MOVEMENT_TIME'.format(i)] = robot.movement_time # 'ROBOT<n>_MOVEMENT_TIME'
            row_fields['ROBOT{0}_WAITING_TIME'.format(i)] = robot.waiting_time   # 'ROBOT<n>_WAITING_TIME'
            row_fields['ROBOT{0}_IDLE_TIME'.format(i)] = robot.idle_time         # 'ROBOT<n>_IDLE_TIME'
            row_fields['ROBOT{0}_DELAY_TIME'.format(i)] = robot.delay_time       # 'ROBOT<n>_DELAY_TIME'

        csv_file.writerow([ row_fields[fn] for fn in field_names ])

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract summary statistics for the runs recorded in given bag files.')

    parser.add_argument('bag_file',
                        nargs='+',
                        help='Path to the bag file(s) of the experiment(s) to plot.')

    parser.add_argument('-o', '--output',
                        default=DEFAULT_CSV_FILENAME,
                        help="Name of the output .csv file")

    args = parser.parse_args()

    bag_files = args.bag_file
    output = args.output

#    pp.pprint(bag_files)

    # Make one, flat list of paths to log files
    bag_paths = []

    for path_arg in bag_files:
        bag_paths.extend(glob.glob(path_arg))

#    print("bag paths: ")
#    pp.pprint(bag_paths)

    parse_stats(bag_paths, output)
