#!/usr/bin/env python

import argparse
from collections import defaultdict
import csv
import glob
import math
import numpy as np
import os
import pprint
import re
import rosbag
import sys
import tf.transformations

import mrta
import mrta.msg

DEFAULT_CSV_FILENAME = 'stats.csv'

field_names = [
    'BAG_FILENAME',
    'DATETIME',
    'MAP',
    'START_CONFIG',
    'MECHANISM',
    'SCENARIO_ID',
    # 'RUN_NUM',
    # 'ATTEMPT_NUM',
    'TOTAL_RUN_TIME',
    'DELIBERATION_TIME',
    'EXECUTION_PHASE_TIME',
    'NAP_TIME',
    'TOTAL_MOVEMENT_TIME',
    'TOTAL_EXECUTION_TIME',
    'TOTAL_WAITING_TIME',
    'TOTAL_IDLE_TIME',
    'TOTAL_DELAY_TIME',
    'TOTAL_DISTANCE',
    'TOTAL_COLLISIONS',
    'TOTAL_DISTANCE_TO_ASSIGNED_MEDIANS',
    'MEDIAN_TASK_IDS',
    'NUM_ANNOUNCE_MSGS',
    'NUM_ANNOUNCE_TASKS',
    'NUM_BID_MSGS',
    'ALLOC_MSG_BYTES',
    'ROBOT1_STARTX',
    'ROBOT1_STARTY',
    'ROBOT1_DISTANCE_TO_ASSIGNED_MEDIAN',
    'ROBOT1_DISTANCE',
    'ROBOT1_MOVEMENT_TIME',
    'ROBOT1_WAITING_TIME',
    'ROBOT1_IDLE_TIME',
    'ROBOT1_DELAY_TIME',
    'ROBOT2_STARTX',
    'ROBOT2_STARTY',
    'ROBOT2_DISTANCE_TO_ASSIGNED_MEDIAN',
    'ROBOT2_DISTANCE',
    'ROBOT2_MOVEMENT_TIME',
    'ROBOT2_WAITING_TIME',
    'ROBOT2_IDLE_TIME',
    'ROBOT2_DELAY_TIME',
    'ROBOT3_STARTX',
    'ROBOT3_STARTY',
    'ROBOT3_DISTANCE_TO_ASSIGNED_MEDIAN',
    'ROBOT3_DISTANCE',
    'ROBOT3_MOVEMENT_TIME',
    'ROBOT3_WAITING_TIME',
    'ROBOT3_IDLE_TIME',
    'ROBOT3_DELAY_TIME',
    'MEAN_MSG_TIME',
    'MAXIMUM_ROBOT_DISTANCE',
    'MECHANISM_SELECTED',
    'MECHANISM_SELECTION_TIME'
]

ROBOT_NAMES = ['robot_1',
               'robot_2',
               'robot_3']

pp = pprint.PrettyPrinter(indent=4)


def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False
    except TypeError:
        return False


class Robot(object):
    def __init__(self):
        self.name = None
        self.startx = None
        self.starty = None
        self.starta = None
        self.distance = 0.0
        self.distance_to_median = None
        self.movement_time = 0.0
        self.idle_time = 0.0
        self.delay_time = 0.0

        self.collisions = 0

        self.exec_phase_begin_stamp = None
        self.exec_phase_end_stamp = None

        # Times at which this robot paused during execution of tasks. Keys are task ids
        # and values are timestamps
        self.pause_times = defaultdict(int)

        # We'll use award messages populate this list and 'SUCCESS' (complete) task
        # status messages to depopulate. At the end of a run it should be empty for each
        # robot. If not, there was a problem (e.g., status messages not recorded).
        self.tasks = []


def _get_intervals(start_event, end_events, messages, label):
    """
    Search messages for all [start_event,end_event] intervals. Return them as
    tuples of the form (start_timestamp, end_timestamp, label).
    """
    intervals = []

    # Return an empty list of intervals of messages is empty
    if not messages:
        return intervals

    # The name of a message's 'event' field depends on its class.
    attr_name = None
    sample_msg = messages[0]
    if sample_msg.__class__.__name__ == '_mrta__ExperimentEvent':
        attr_name = 'event'
    elif sample_msg.__class__.__name__ == '_mrta__TaskStatus':
        attr_name = 'status'
    else:
        # Should actually raise an error here
        print('Unrecognized message class: {0}'.format(sample_msg.__class__.__name__))
        return intervals

    last_start_stamp = None
    for message in messages:
        event = message.__getattribute__(attr_name)
        if event == start_event:
            last_start_stamp = message.header.stamp.secs + (message.header.stamp.nsecs/1000000000.)
            continue
        elif event in end_events:
            if not last_start_stamp:
                print("count_interval_time(): {0} not preceded by {1}!".format(event, start_event))
                continue
            else:
                end_stamp = message.header.stamp.secs + (message.header.stamp.nsecs/1000000000.)
                intervals.append([last_start_stamp, end_stamp, label])
                last_start_stamp = None

    # print("total: {0}".format(interval_times))
    return intervals


def _normalize_times(intervals, start_time):
    for i in range(len(intervals)):
        intervals[i][0] -= start_time
        intervals[i][1] -= start_time

    return intervals


def _get_idle_intervals(intervals, exec_phase_intervals, robot_name):

    idle_intervals = []

    # Make sure intervals are sorted
    intervals = sorted(intervals, key=lambda x: x[0])
    # print("{0} intervals: {1}".format(robot_name, pp.pformat(intervals)))

    # For each interval in 'exec_phase_intervals', run through 'intervals' and see if there is a
    # gap between 'moving' or 'waiting' interval end times and the end of the execution phase interval

    for exec_phase_interval in exec_phase_intervals:

        exec_phase_begin_time = exec_phase_interval[0]
        exec_phase_end_time = exec_phase_interval[1]

        # last_exec_interval = None
        last_exec_interval = exec_phase_interval

        for interval in intervals:

            # Make sure this interval falls within the given execution phase
            interval_begin_time = interval[0]
            interval_end_time = interval[1]

            start_valid = end_valid = False

            # Interval begins or ends within this exec phase interval
            if exec_phase_begin_time < interval_begin_time < exec_phase_end_time:
                start_valid = True

            if exec_phase_begin_time < interval_end_time < exec_phase_end_time:
                end_valid = True

            if start_valid or end_valid:
                last_exec_interval = interval

        # print("exec_phase_interval: {0}".format(pp.pformat(exec_phase_interval)))
        # print("last_exec_interval: {0}".format(pp.pformat(last_exec_interval)))

        if last_exec_interval and last_exec_interval[1] <= exec_phase_end_time:
            if last_exec_interval == exec_phase_interval:
                # print("appending [{0}, {1}, 'idle']".format(last_exec_interval[0], exec_phase_end_time))
                idle_intervals.append([last_exec_interval[0], exec_phase_end_time, 'idle'])
            else:
                # print("appending [{0}, {1}, 'idle']".format(last_exec_interval[1], exec_phase_end_time))
                idle_intervals.append([last_exec_interval[1], exec_phase_end_time, 'idle'])

    return idle_intervals


def count_interval_times(start_event, end_event, messages):
    """
    Count the time in all [start_event,end_event] intervals and return the sum.
    """
    interval_times = 0.0

    last_start_stamp = None
    last_start_time = None

    # If messages is empty, return 0
    if not messages:
        return interval_times

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
        if event == start_event:
            # print("setting last_start_stamp to: {0}".format(pp.pformat(message.header.stamp)))
            last_start_stamp = message.header.stamp
            # print("message: {0}".format(pp.pformat(message)))
            # print("setting last_start_time to: {0}".format(pp.pformat(message.time)))
            # last_start_stamp = message.time
            continue
        elif event == end_event:
            if not last_start_stamp:
                print("count_interval_time(): {0} not preceded by a {1}!".format(end_event, start_event))
                continue
            else:
                # print("start time: {0}, end time: {1}".format(pp.pformat(last_start_stamp), pp.pformat(message.header.stamp)))
                if message.header.stamp < last_start_stamp:
                    print("Interval end time is earlier than its start time. Something is wrong!")
                interval_time = message.header.stamp - last_start_stamp
                interval_secs = interval_time.secs + (interval_time.nsecs/1000000000.)
                # print("{0}--{1}=={2}".format(start_event, end_event, interval_secs))
                interval_times += interval_secs
                last_start_stamp = None

    # print("total: {0}".format(interval_times))
    return interval_times


def parse_stats(bag_paths, output):

    # Open the CSV file for writing
    csv_file = csv.writer(open(output, 'wb'))
    csv_file.writerow(field_names)

    dt_re = re.compile('(.*)\.bag')
    # median_distance_re = re.compile('.*\[(\w+)\].*\[(\w+)\] == \[(\d+\.\d+)\]')
    median_distance_re = re.compile('.*\[(\w+)\].*\[(\w+)\] == \[(\d+\.\d+([eE][+-]?\d+)?)\]')
    median_task_ids_re = re.compile('median task ids: \[(.*)\]')

    for i, bag_path in enumerate(bag_paths):
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

        print("bag_filename: {0}".format(bag_filename))
        (map, start_config, mechanism, scenario_id, remainder) = bag_filename.split('__')

        # Date/time
        dt_match = dt_re.search(remainder)
        row_fields['DATETIME'] = dt_match.group(1)

        # 'START_CONFIG', 'MECHANSIM', 'SCENARIO_ID'
#        row_fields.extend([map, start_config, mechanism, scenario_id])
        row_fields['MAP'] = map
        row_fields['START_CONFIG'] = start_config
        row_fields['MECHANISM'] = mechanism
        row_fields['SCENARIO_ID'] = scenario_id

        run_msgs = defaultdict(list)

        try:
            for topic, msg, msg_time in bag.read_messages():
                # msg.header.stamp = msg_time
                run_msgs[topic].append(msg)
        except:
            print("Couldn't read messages from {0}!".format(bag_path))
            print(sys.exc_info()[0])
            continue

        exp_msgs = []
        experiment_finished = False
        for msg in run_msgs['/experiment']:
            if msg.event == mrta.msg.ExperimentEvent.END_EXPERIMENT:
                experiment_finished = True
            exp_msgs.append(msg)

        # if 'END_EXPERIMENT' not in exp_msgs:
        if not experiment_finished:
            print("Experiment timed out! Skipping...")
            continue

        # MEAN_MSG_TIME
        row_fields['MEAN_MSG_TIME'] = np.mean([float(m.value) for m in run_msgs['/debug'] if m.key.startswith('status')])
#        for debug_msg in run_msgs['/delay']:
#            pass

#        print('TOTAL_RUN_TIME:')
        total_run_time = count_interval_times(mrta.msg.ExperimentEvent.BEGIN_EXPERIMENT,
                                              mrta.msg.ExperimentEvent.END_EXPERIMENT,
                                              exp_msgs)
        row_fields['TOTAL_RUN_TIME'] = total_run_time  # 'TOTAL_RUN_TIME'

#        print('DELIBERATION_TIME:')
        delib_time = count_interval_times(mrta.msg.ExperimentEvent.BEGIN_ALLOCATION,
                                          mrta.msg.ExperimentEvent.END_ALLOCATION,
                                          exp_msgs)
        row_fields['DELIBERATION_TIME'] = delib_time  # 'DELIBERATION_TIME'

#        print('EXECUTION_PHASE_TIME:')
        exec_phase_time = count_interval_times(mrta.msg.ExperimentEvent.BEGIN_EXECUTION,
                                               mrta.msg.ExperimentEvent.END_EXECUTION,
                                               exp_msgs)
        row_fields['EXECUTION_PHASE_TIME'] = exec_phase_time  # 'EXECUTION_PHASE_TIME'

        # Get execution intervals
        exec_intervals = _get_intervals(mrta.msg.ExperimentEvent.BEGIN_EXECUTION,
                                        [mrta.msg.ExperimentEvent.END_EXECUTION],
                                        exp_msgs,
                                        'execution')

        # 'Nap' time
        nap_time = count_interval_times(mrta.msg.ExperimentEvent.END_EXECUTION,
                                        mrta.msg.ExperimentEvent.BEGIN_ALLOCATION,
                                        exp_msgs)
        row_fields['NAP_TIME'] = nap_time  # 'NAP_TIME'

        selection_time = count_interval_times(mrta.msg.ExperimentEvent.BEGIN_SELECT_MECHANISM,
                                              mrta.msg.ExperimentEvent.END_SELECT_MECHANISM,
                                              exp_msgs)
        row_fields['MECHANISM_SELECTION_TIME'] = selection_time  # 'MECHANISM_SELECTION_TIME'

        exp_begin_stamp = None
        exp_begin_time = None
        exp_end_execution_stamp = None
        for msg in exp_msgs:
            if msg.event == mrta.msg.ExperimentEvent.BEGIN_EXPERIMENT:
                exp_begin_stamp = msg.header.stamp
                exp_begin_time = exp_begin_stamp.secs + (exp_begin_stamp.nsecs/1000000000.)
            if msg.event == mrta.msg.ExperimentEvent.END_EXECUTION:
                exp_end_execution_stamp = msg.header.stamp

        _normalize_times(exec_intervals, exp_begin_time)

        total_movement_time = 0.0
        total_execution_time = 0.0
        total_waiting_time = 0.0
        total_idle_time = 0.0
        total_delay_time = 0.0
        total_distance = 0.0
        total_collisions = 0
        total_distance_to_assigned_medians = 0.0

        robots = {}

        for r_name in ROBOT_NAMES:
            robot = Robot()
            robot.name = r_name
            robots[r_name] = robot

        for award_msg in run_msgs['/tasks/award']:
            robot = robots[award_msg.robot_id]

            for task in award_msg.tasks:
                # print("{0} was awarded task {1}".format(award_msg.robot_id, task.task.task_id))
                robot.tasks.append(task.task.task_id)
                #
                # print("{0}'s task list: {1}".format(award_msg.robot_id, pp.pformat(robots[award_msg.robot_id].tasks)))

        inconsistent_tasks = False

        # per-robot stats
        for r_name in ROBOT_NAMES:
            robot = robots[r_name]

            print("{0}'s task list: {1}".format(r_name, pp.pformat(robot.tasks)))

            # Distance travelled
            robot.distance = 0.0
            last_pose = None
            for amcl_msg in run_msgs['/{0}/amcl_pose'.format(r_name)]:

                amcl_pose = amcl_msg.pose.pose

                if not robot.startx:
                    robot.startx = amcl_pose.position.x

                if not robot.starty:
                    robot.starty = amcl_pose.position.y

                if not robot.starta:

                    quat = (amcl_pose.orientation.x,
                            amcl_pose.orientation.y,
                            amcl_pose.orientation.z,
                            amcl_pose.orientation.w)
                    euler = tf.transformations.euler_from_quaternion(quat)
                    robot.starta = euler[2] * 180. / math.pi

                if amcl_pose.position.x == 0 and amcl_pose.position.y == 0:
                    continue

                # print("last_pose: {0}".format(last_pose))
                # print("amcl_pose: {0}".format(last_pose))

                if last_pose is None:
                    last_pose = amcl_pose
                    continue

                pos_delta = math.hypot(amcl_pose.position.x - last_pose.position.x,
                                       amcl_pose.position.y - last_pose.position.y)
                robot.distance += pos_delta
                last_pose = amcl_pose

            print("{0} start pos: ({1}, {2}, {3})".format(r_name, robot.startx, robot.starty, robot.starta))

#            print "{0} distance: {1}".format(r_name, robot.distance)

            # Distance to the robot's assigned (p-)median
            for debug_msg in run_msgs['/debug']:
                if not robot.distance_to_median:
                    # print "finding median distance for {0}".format(robot.name)
                    if debug_msg.key == 'auctioneer-median-distance':
                        print "median distance message: {0}".format(debug_msg.value)
                        matches = re.search(median_distance_re, debug_msg.value)
                        msg_robot_id = matches.group(1)
                        msg_median_task_id = matches.group(2)
                        msg_median_distance = float(matches.group(3))

                        if msg_robot_id == robot.name:
                            robot.distance_to_median = msg_median_distance
                            # print "median distance found for {0}".format(robot.name)

            r_status_msgs = [m for m in run_msgs['/tasks/status'] if m.robot_id == r_name]

            # print('{0} movement time:'.format(r_name))
            # Movement time
            robot.movement_time = count_interval_times(mrta.msg.TaskStatus.MOVING,
                                                       mrta.msg.TaskStatus.ARRIVED,
                                                       r_status_msgs)

            robot.movement_time += count_interval_times(mrta.msg.TaskStatus.MOVING,
                                                        mrta.msg.TaskStatus.FAILURE,
                                                        r_status_msgs)

#            print('{0} execution time:'.format(r_name))
            # Execution time
            robot.execution_time = count_interval_times(mrta.msg.TaskStatus.BEGIN,
                                                        mrta.msg.TaskStatus.SUCCESS,
                                                        r_status_msgs)

#            print('{0} waiting time:'.format(r_name))
            # Waiting time
            robot.waiting_time = count_interval_times(mrta.msg.TaskStatus.ARRIVED,
                                                      mrta.msg.TaskStatus.BEGIN,
                                                      r_status_msgs)

#            print('{0} delay time:'.format(r_name))
            # Delay time
            robot.delay_time = count_interval_times(mrta.msg.TaskStatus.PAUSE,
                                                    mrta.msg.TaskStatus.RESUME,
                                                    r_status_msgs)

            for status_msg in r_status_msgs:

                if status_msg.status == mrta.msg.TaskStatus.MOVING and robot.exec_phase_begin_stamp is None:
                    robot.exec_phase_begin_stamp = status_msg.header.stamp

                if status_msg.status == mrta.msg.TaskStatus.SUCCESS or status_msg == mrta.msg.TaskStatus.FAILURE:
                    robot.exec_phase_end_stamp = status_msg.header.stamp

                    # On success, we should remove this task (id) from the robot's task list
                    if status_msg.status == mrta.msg.TaskStatus.SUCCESS:
                        print "Removing task {0}".format(status_msg.task_id)
                        try:
                            robot.tasks.remove(status_msg.task_id)
                        except ValueError:
                            print("Robot 'successfully completed' a task that was not in its agenda.")
                            inconsistent_tasks = True

                if status_msg.status == mrta.msg.TaskStatus.PAUSE:
                    robot.collisions += 1

            if robot.tasks:
                print("{0} was awarded tasks {1} but there is no record of successful completion!".format(r_name, pp.pformat(robot.tasks)))
                inconsistent_tasks = True

            if not robot.exec_phase_begin_stamp or not robot.exec_phase_end_stamp:
                print("Can not determine beginning or end time of {0}'s exec phase!".format(r_name))
                continue

            # 'moving' intervals
            moving_intervals = _get_intervals(mrta.msg.TaskStatus.MOVING,
                                              [mrta.msg.TaskStatus.PAUSE,
                                               mrta.msg.TaskStatus.ARRIVED,
                                               mrta.msg.TaskStatus.FAILURE],
                                              r_status_msgs,
                                              'moving')
            _normalize_times(moving_intervals, exp_begin_time)

            # 'waiting' intervals
            waiting_intervals = _get_intervals(mrta.msg.TaskStatus.ARRIVED,
                                               [mrta.msg.TaskStatus.BEGIN,
                                                mrta.msg.ExperimentEvent.BEGIN_ALLOCATION],
                                               r_status_msgs,
                                               'waiting')
            _normalize_times(waiting_intervals, exp_begin_time)

            # idle_time_diff = exp_end_execution_stamp - robot.exec_phase_end_stamp
            # robot.idle_time = (idle_time_diff.secs + idle_time_diff.nsecs/1000000000.)

            # Count idle time(s)
            idle_intervals = _get_idle_intervals(moving_intervals + waiting_intervals,
                                                 exec_intervals,
                                                 r_name)

            print("idle_intervals: {0}".format(pp.pformat(idle_intervals)))

            for idle_interval in idle_intervals:
                robot.idle_time += idle_interval[1] - idle_interval[0]

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
            print("{0} distance: {1}".format(r_name, robot.distance))
            total_collisions += robot.collisions
            if robot.distance_to_median:
                total_distance_to_assigned_medians += robot.distance_to_median

        if inconsistent_tasks:
            print("A robot either did not complete tasks it was awarded or vice versa. Skipping this run!")
            continue

        print("total_collisions: {0}".format(total_collisions))
        print("total_distance: {0}".format(total_distance))

        row_fields['TOTAL_MOVEMENT_TIME'] = total_movement_time      # 'TOTAL_MOVEMENT_TIME'
        row_fields['TOTAL_EXECUTION_TIME'] = total_execution_time    # 'TOTAL_EXECUTION_TIME'
        row_fields['TOTAL_WAITING_TIME'] = total_waiting_time        # 'TOTAL_WAITING_TIME'
        row_fields['TOTAL_IDLE_TIME'] = total_idle_time              # 'TOTAL_IDLE_TIME'
        row_fields['TOTAL_DELAY_TIME'] = total_delay_time            # 'TOTAL_DELAY_TIME'
        row_fields['TOTAL_DISTANCE'] = total_distance                # 'TOTAL_DISTANCE'
        row_fields['TOTAL_COLLISIONS'] = total_collisions            # 'TOTAL_COLLISIONS'
        row_fields['TOTAL_DISTANCE_TO_ASSIGNED_MEDIANS'] = total_distance_to_assigned_medians  # 'TOTAL_COLLISIONS'

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

        # for r_name in ROBOT_NAMES:
        #     robot = robots[r_name]

        p_median_task_ids = None
        # p-median task ids
        for debug_msg in run_msgs['/debug']:
            if p_median_task_ids:
                break

            if debug_msg.key == 'auctioneer-median-ids':
                print "median task ids message: {0}".format(debug_msg.value)
                matches = re.search(median_task_ids_re, debug_msg.value)
                if matches:
                    p_median_task_ids_csv = matches.group(1)

                    # Sigh.. this is a comma-separated string. If we output it literally it will
                    # mess with the CSV format. Turn it into a '-'-separated string
                    p_median_task_ids = '-'.join(p_median_task_ids_csv.split(','))

        row_fields['MEDIAN_TASK_IDS'] = p_median_task_ids

        # Selected mechanism, if any
        row_fields['MECHANISM_SELECTED'] = None
        for debug_msg in run_msgs['/debug']:
            if debug_msg.key == 'auctioneer-selected-mechanism':
                row_fields['MECHANISM_SELECTED'] = debug_msg.value

        for j in range(1, 4):
            r_name = 'robot_{0}'.format(j)
            robot = robots[r_name]

            row_fields['ROBOT{0}_STARTX'.format(j)] = robot.startx                    # 'ROBOT<n>_STARTX'
            row_fields['ROBOT{0}_STARTY'.format(j)] = robot.starty                    # 'ROBOT<n>_STARTY'
            row_fields['ROBOT{0}_DISTANCE_TO_ASSIGNED_MEDIAN'.format(j)] = robot.distance_to_median  # 'ROBOT<n>_DISTANCE_TO_MEDIAN'
            row_fields['ROBOT{0}_DISTANCE'.format(j)] = robot.distance                # 'ROBOT<n>_DISTANCE'
            row_fields['ROBOT{0}_MOVEMENT_TIME'.format(j)] = robot.movement_time      # 'ROBOT<n>_MOVEMENT_TIME'
            row_fields['ROBOT{0}_WAITING_TIME'.format(j)] = robot.waiting_time        # 'ROBOT<n>_WAITING_TIME'
            row_fields['ROBOT{0}_IDLE_TIME'.format(j)] = robot.idle_time              # 'ROBOT<n>_IDLE_TIME'
            row_fields['ROBOT{0}_DELAY_TIME'.format(j)] = robot.delay_time            # 'ROBOT<n>_DELAY_TIME'

        row_fields['MAXIMUM_ROBOT_DISTANCE'] = max(row_fields['ROBOT1_DISTANCE'],
                                                   row_fields['ROBOT2_DISTANCE'],
                                                   row_fields['ROBOT3_DISTANCE'])

        # Round float values to 6 digits of precision
        for key in row_fields.keys():
            if is_number(row_fields[key]) and key not in ['START_CONFIG']:
                row_fields[key] = round(row_fields[key], 6)

        csv_file.writerow([row_fields[fn] for fn in field_names])

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

    # pp.pprint(bag_files)

    # Make one, flat list of paths to log files
    bag_paths = []

    for path_arg in bag_files:
        bag_paths.extend(glob.glob(path_arg))
    # print("bag paths: ")
    # pp.pprint(bag_paths)

    parse_stats(bag_paths, output)
