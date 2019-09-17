#!/usr/bin/env python

import argparse
from collections import defaultdict
import glob
import matplotlib.pyplot as plt
import mrta
import mrta.msg
import numpy as np
import os
import os.path
import pandas as pd
import rosbag
import sys

# For debugging
import pprint
pp = pprint.PrettyPrinter(indent=4)

# Some global constants
robot_names = ['robot_1',
               'robot_2',
               'robot_3']

colors = {'moving': [0.0, 1.0, 0.0],          # 'green'
          'waiting': [1.0, 0.0, 0.0],         # 'red'
          #'waiting': [0.0, 1.0, 0.0],         # 'red'
          'delay': [0.0, 0.0, 1.0],           # 'blue'
          'idle': [0.5, 0.0, 1.0],            # violet
          'deliberation': [0.7, 0.7, 0.7],    # grey
          'nap': [1.0, 0.796, 0.859],         # pink
          #'execution': [0.0, 0.0, 0.0, 0.0],  # transparent
          'execution': [1.0, 1.0, 0.0],  # transparent
	  'task_completion':[1.0, 0.5, 0.0]}  # orange


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
                # print("count_interval_time(): {0} not preceded by {1}!".format(event, start_event))
                continue
            else:
                end_stamp = message.header.stamp.secs + (message.header.stamp.nsecs/1000000000.)
                intervals.append([last_start_stamp, end_stamp, label])
                last_start_stamp = None

    # print("total: {0}".format(interval_times))
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


def _normalize_times(intervals, start_time):
    for i in range(len(intervals)):
        intervals[i][0] -= start_time
        intervals[i][1] -= start_time

    return intervals


def plot_timeline(run_msgs, plot_title, plot_filename):
    print "Plotting {0}".format(plot_filename)

    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    # plt.title(plot_title)
    ax.set_yticklabels(robot_names[::-1])
    ax.set_xlabel('seconds')

    # Get the overall 'experiment' interval
    exp_interval = _get_intervals(mrta.msg.ExperimentEvent.BEGIN_EXPERIMENT,
                                  [mrta.msg.ExperimentEvent.END_EXPERIMENT],
                                  run_msgs['/experiment'],
                                  'experiment')[0]
    # print("Experiment: {0}".format(pp.pformat(exp_interval)))
    exp_start_time = exp_interval[0]
    exp_end_time = exp_interval[1]

    plt.xlim(0, (exp_end_time-exp_start_time)+5)
    # plt.xlim(0, 200)
    ax.set_ylim(0, 40)
    ax.set_yticks([10, 20, 30])
    


    # Get deliberation intervals
    delib_intervals = _get_intervals(mrta.msg.ExperimentEvent.BEGIN_ALLOCATION,
                                     [mrta.msg.ExperimentEvent.END_ALLOCATION],
                                     run_msgs['/experiment'],
                                     'deliberation')
    _normalize_times(delib_intervals, exp_start_time)
    # print("delib intervals: {0}".format(pp.pformat(delib_intervals)))

    # Get execution intervals
    exec_intervals = _get_intervals(mrta.msg.ExperimentEvent.BEGIN_EXECUTION,
                                    [mrta.msg.ExperimentEvent.END_EXECUTION],
                                    run_msgs['/experiment'],
                                    'execution')
    _normalize_times(exec_intervals, exp_start_time)

    # Get 'nap' intervals
    nap_intervals = _get_intervals(mrta.msg.ExperimentEvent.END_EXECUTION,
                                   [mrta.msg.ExperimentEvent.BEGIN_ALLOCATION],
                                   run_msgs['/experiment'],
                                   'nap')
    _normalize_times(nap_intervals, exp_start_time)

    robot_msgs = defaultdict(list)
    for msg in run_msgs['/tasks/status']:
        robot_msgs[msg.robot_id].append(msg)

    # Go through the robots in reverse order because the y-axis starts
    # at the bottom of the plot, but we'd like robots 1-3 to be plotted
    # top to bottom.
    y_pos = 7
    for robot_name in robot_names[::-1]:

        # print("{0} messages: {1}".format(robot_name, pp.pformat(robot_msgs[robot_name])))

        moving_intervals = _get_intervals(mrta.msg.TaskStatus.MOVING,
                                          [mrta.msg.TaskStatus.PAUSE,
                                           mrta.msg.TaskStatus.ARRIVED,
                                           mrta.msg.TaskStatus.FAILURE],
                                          robot_msgs[robot_name],
                                          'moving')
        _normalize_times(moving_intervals, exp_start_time)
        # print("moving intervals: {0}".format(pp.pformat(moving_intervals)))

        delay_intervals = _get_intervals(mrta.msg.TaskStatus.PAUSE,
                                         [mrta.msg.TaskStatus.RESUME],
                                         robot_msgs[robot_name],
                                         'delay')
        _normalize_times(delay_intervals, exp_start_time)
        # print("delay intervals: {0}".format(pp.pformat(delay_intervals)))

        waiting_intervals = _get_intervals(mrta.msg.TaskStatus.ARRIVED,
                                           [mrta.msg.TaskStatus.BEGIN,
                                            mrta.msg.ExperimentEvent.BEGIN_ALLOCATION],
                                           robot_msgs[robot_name],
                                           'waiting')
        _normalize_times(waiting_intervals, exp_start_time)
        # print("waiting intervals: {0}".format(pp.pformat(waiting_intervals)))

	# Get 'task_completion_intervals' 
	# 'task_completion' intervals
	task_completion_intervals = _get_intervals(mrta.msg.TaskStatus.MOVING,
					          [mrta.msg.TaskStatus.SUCCESS,
						   mrta.msg.ExperimentEvent.BEGIN_ALLOCATION],
                                          	  #[mrta.msg.TaskStatus.PAUSE,
                                           	  # mrta.msg.TaskStatus.ARRIVED,
                                           	  # mrta.msg.TaskStatus.FAILURE,
						  #mrta.msg.TaskStatus.SUCCESS,
						  #mrta.msg.ExperimentEvent.BEGIN_ALLOCATION],
                                                  robot_msgs[robot_name],
                                                  'task_completion')
	_normalize_times(task_completion_intervals, exp_start_time)

	all_robot_intervals = delib_intervals + task_completion_intervals  

        #all_robot_intervals = delib_intervals + nap_intervals + moving_intervals + \
         #                     delay_intervals + waiting_intervals + exec_intervals

        # print("all {0} intervals: {1}".format(robot_name, pp.pformat(all_robot_intervals)))

        all_robot_intervals = sorted(all_robot_intervals, key=lambda x: x[0])

        # print("sorted {0} intervals: {1}".format(robot_name, pp.pformat(all_robot_intervals)))

        # Insert idle intervals
        idle_intervals = _get_idle_intervals(all_robot_intervals, exec_intervals, robot_name)
        # print("idle_intervals: {0}".format(pp.pformat(idle_intervals)))
        all_robot_intervals += idle_intervals
        all_robot_intervals = sorted(all_robot_intervals, key=lambda x: x[0])

        bars = ax.broken_barh([(x[0], x[1]-x[0]) for x in all_robot_intervals], (y_pos, 6),
                              facecolors=[colors[x[2]] for x in all_robot_intervals])
        # print("bars: {0}".format(bars))

        y_pos += 10

    # Bar labels
    # plt.tick_params(axis='x', which='major', labelsize=18)
    # plt.xticks(x_pos, mechanisms)
    ymin, ymax = plt.ylim()
    # plt.legend(bars, [], loc='lower center', ncol=2, fancybox=True, shadow=True)

    plt.tight_layout()

    plt.savefig(plot_filename, bbox_inches='tight', dpi=200)
    plt.close()


def plot_timelines(bag_paths):

    # Create the './timelines' directory if it doesn't already exist
    timeline_dir = './timelines'
    if not os.path.exists(timeline_dir):
        os.makedirs(timeline_dir)

    for bag_path in bag_paths:
        print("Reading {0}".format(bag_path))

        bag = None
        try:
            bag = rosbag.Bag(bag_path)
        except:
            print("Couldn't open {0} for reading!".format(bag_path))
            continue

        bag_filename = os.path.basename(bag_path)
        (map, start_config, mechanism, task_file, remainder) = bag_filename.split('__')

        run_msgs = defaultdict(list)
        try:
            for topic, msg, msg_time in bag.read_messages():
                try:
                    msg.header.stamp = msg_time
                except AttributeError:
                    continue
                run_msgs[topic].append(msg)
        except:
            print("Couldn't read messages from {0}!".format(bag_path))
            print(sys.exc_info()[:2])
            continue

        experiment_finished = False
        for msg in run_msgs['/experiment']:
            if msg.event == mrta.msg.ExperimentEvent.END_EXPERIMENT:
                experiment_finished = True

        if not experiment_finished:
            print("Experiment timed out! Skipping...")
            continue

        bag_basename = bag_filename.replace('.bag', '')
        bag_basename = bag_basename.replace('.yaml', '')

        # print("bag_basename: {0}".format(bag_basename))
        plot_filename = 'timeline__' + bag_basename + '.pdf'
        plot_filename = os.path.join(timeline_dir, plot_filename)

        task_file = task_file.replace('.txt', '')
        task_file = task_file.replace('.yaml', '')

        remainder = remainder.replace('.bag', '')
        plot_title = '"{0}": {1}, {2} start ({3})'.format(task_file, mechanism, start_config, remainder)

        plot_timeline(run_msgs, plot_title, plot_filename)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot a per-robot timeline of events for given experiments (bags).')

    parser.add_argument('bag_file',
                        nargs='+',
                        help='Path to the bag file of the experiment to plot.')

    args = parser.parse_args()

    bag_files = args.bag_file

#    pp.pprint(bag_files)

    # Make one, flat list of paths to log files
    bag_paths = []

    for path_arg in bag_files:
        bag_paths.extend(glob.glob(path_arg))

    print("bag paths: ")
    pp.pprint(bag_paths)

    plot_timelines(bag_paths)
