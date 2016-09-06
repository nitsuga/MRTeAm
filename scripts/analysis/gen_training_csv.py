#!/usr/bin/env python

import argparse
from collections import defaultdict
import csv
import glob
from itertools import chain, combinations
import math
import numpy as np
import os
from os import listdir
from os.path import isdir, isfile, join
import pandas as pd
import pprint
import re
import signal
import subprocess
import sys
import time
import yaml

import igraph

import rospkg
import rospy
import rosbag

import mrta
import mrta.msg
import mrta.mrta_planner_proxy


IN_CSV_FILENAME = 'stats.csv'
OUT_DIST_CSV_FILENAME = 'team_distance.csv'
OUT_RUN_TIME_CSV_FILENAME = 'run_time.csv'
OUT_EXECUTION_PHSE_TIME_CSV_FILENAME = 'execution_phase_time.csv'
OUT_MINIMAX_CSV_FILENAME = 'minimax_distance.csv'

OUT_FIELDNAMES = ['TOTAL_DISTANCE_TO_MEDIANS',
                  'TOTAL_DISTANCE_TO_ALL_MEDIANS',
                  'MEDIAN_SPREAD',
                  'MAX_DISTANCE_TO_MEDIAN',
                  'MIN_DISTANCE_TO_MEDIAN',
                  'TEAM_DIAMETER',
                  'MEDIAN_ASSIGNMENT_SPREAD',
                  'ROBOT1_DISTANCE_TO_MEDIAN',
                  'ROBOT1_DISTANCE_TO_ALL_MEDIANS',
                  'ROBOT1_STARTX',
                  'ROBOT1_STARTY',
                  'ROBOT2_DISTANCE_TO_MEDIAN',
                  'ROBOT2_DISTANCE_TO_ALL_MEDIANS',
                  'ROBOT2_STARTX',
                  'ROBOT2_STARTY',
                  'ROBOT3_DISTANCE_TO_MEDIAN',
                  'ROBOT3_DISTANCE_TO_ALL_MEDIANS',
                  'ROBOT3_STARTX',
                  'ROBOT3_STARTY',
                  'MECHANISM']

ROBOT_NAMES = ['robot1', 'robot2', 'robot3']

# Default location of task (target point) files
TASK_FILES_DEFAULT = "{0}/task_files".format(rospkg.RosPack().get_path('mrta_auctioneer'))

# Key is a task filename, value is a dict, where
#  key is a task_id, value is an (x, y) pair of coordinates
target_point_configs = {}

pp = pprint.PrettyPrinter(indent=4)


# Paths to ROS binaries
ROS_HOME = '/opt/ros/indigo'
ROSLAUNCH = "{0}/bin/roslaunch".format(ROS_HOME)

LAUNCH_PKG = 'mrta'
LAUNCHFILE = 'stage_dummy_robot.launch'
DUMMY_ROBOT_NAME = 'robot_0'
MAP_FILE = 'smartlab_ugv_arena_v2.png'
WORLD_FILE = 'smartlab_ugv_arena_dummy_robot.world'
#NOGUI_FLAG = ''
NOGUI_FLAG = '-g'

planner_proxy = None
main_proc = None


def start_ros():
    global main_proc, planner_proxy

    try:

        main_proc = subprocess.Popen([ROSLAUNCH,
                                      LAUNCH_PKG,
                                      LAUNCHFILE,
                                      "map_file:={0}".format(MAP_FILE),
                                      "world_file:={0}".format(WORLD_FILE),
                                      "dummy_robot_name:={0}".format(DUMMY_ROBOT_NAME),
                                      "nogui_flag:={0}".format(NOGUI_FLAG)])

        time.sleep(5)

        planner_proxy = mrta.mrta_planner_proxy.PlannerProxy(DUMMY_ROBOT_NAME)

        node_name = 'gen_median'
        rospy.loginfo("Starting node '{0}'...".format(node_name))
        rospy.init_node(node_name)

    except:
        print("Couldn't start ROS: {0}".format(sys.exc_info()[0]))
        stop_ros()
        sys.exit(1)


def stop_ros():
    global main_proc
    try:
        # proc.terminate()
        main_proc.send_signal(signal.SIGINT)
        main_proc.wait()
    except rospy.exceptions.ROSException as e:
        print(e)
        print("Error: {0}".format(sys.exc_info()[0]))


def sig_handler(sig, frame):
    """ Terminate all child processes when we get a SIGINT """
    if sig == signal.SIGINT:
        stop_ros()
        print("Shutting down...")
        sys.exit(0)

signal.signal(signal.SIGINT, sig_handler)


def _read_points(task_file):
    points = {}

    # YAML config file
    if task_file.name.endswith('yaml'):
        yaml_tasks = yaml.load(task_file)

        for yaml_task in yaml_tasks:
            task_loc = yaml_task['location']
            x, y = float(task_loc['x']), float(task_loc['y'])
            points[yaml_task['task_id']] = (x, y)

    return points


def read_point_configs(task_dir):
    global target_point_configs

    task_filenames = [f for f in listdir(task_dir) if isfile(join(task_dir, f))]

    for task_filename in task_filenames:
        task_file_path = join(task_dir, task_filename)

        print "Reading points from {0}...".format(task_file_path)
        task_file = open(task_file_path, "rb")
        target_point_configs[task_filename] = _read_points(task_file)


def write_training_files(in_file, out_dist, out_run_time, out_execution_phase_time, out_minimax, task_dir):
    global planner_proxy
    try:

        read_point_configs(task_dir)

        stats_frame = pd.read_csv(in_file)

        # Only look at PSI and SSI for now
        psi_ssi = stats_frame[stats_frame.MECHANISM.isin(['PSI', 'SSI'])]

        out_dist_csv = csv.writer(open(out_dist, 'wb'))
        out_dist_csv.writerow(OUT_FIELDNAMES)

        out_run_time_csv = csv.writer(open(out_run_time, 'wb'))
        out_run_time_csv.writerow(OUT_FIELDNAMES)

        out_execution_phase_time_csv = csv.writer(open(out_execution_phase_time, 'wb'))
        out_execution_phase_time_csv.writerow(OUT_FIELDNAMES)

        out_minimax_csv = csv.writer(open(out_minimax, 'wb'))
        out_minimax_csv.writerow(OUT_FIELDNAMES)

        # for name, group in stats_frame.groupby([stats_frame.ROBOT1_STARTX, stats_frame.ROBOT1_STARTY]):

        # Group by start positions
        #for name, group in psi_ssi.groupby([stats_frame.ROBOT1_STARTX, stats_frame.ROBOT1_STARTY]):

        # Group by task file
        for name, group in psi_ssi.groupby([stats_frame.TASK_FILE]):

            print name

            # # Only record a training instance if runs for all three mechanisms (PSI, SSI, OSI) succeeded
            # if group.ROBOT1_STARTX.count() < 3:
            #     continue

            # Only record a training instance if runs for both mechanisms (PSI, SSI) succeeded
            if group.ROBOT1_STARTX.count() < 2:
                continue

            # We only have two rows in this group (one per PSI and SSI)
            first_row = group.iloc[0]
            second_row = group.iloc[1]

            first_row_max_dist = max(first_row.ROBOT1_DISTANCE, first_row.ROBOT2_DISTANCE, first_row.ROBOT3_DISTANCE)
            second_row_max_dist = max(second_row.ROBOT1_DISTANCE, second_row.ROBOT2_DISTANCE, second_row.ROBOT3_DISTANCE)

            minimax_row = None
            minimax_loser_row = None

            if first_row_max_dist < second_row_max_dist:
                minimax_row = first_row
                minimax_loser_row = second_row
            else:
                minimax_row = second_row
                minimax_loser_row = first_row

            minimax_margin = abs(first_row_max_dist - second_row_max_dist)

            # find the rows of the "winners" for both distance and time
            min_dist_idx = group.TOTAL_DISTANCE.idxmin()
            min_dist_row = stats_frame.ix[min_dist_idx]
            max_dist_idx = group.TOTAL_DISTANCE.idxmax()
            max_dist_row = stats_frame.ix[max_dist_idx]

            # min_run_time_idx = group.TOTAL_MOVEMENT_TIME.idxmin()
            # min_run_time_row = stats_frame.ix[min_run_time_idx]
            # max_run_time_idx = group.TOTAL_MOVEMENT_TIME.idxmax()
            # max_run_time_row = stats_frame.ix[max_run_time_idx]

            min_run_time_idx = group.TOTAL_RUN_TIME.idxmin()
            min_run_time_row = stats_frame.ix[min_run_time_idx]
            max_run_time_idx = group.TOTAL_RUN_TIME.idxmax()
            max_run_time_row = stats_frame.ix[max_run_time_idx]

            min_execution_phase_time_idx = group.EXECUTION_PHASE_TIME.idxmin()
            min_execution_phase_time_row = stats_frame.ix[min_execution_phase_time_idx]
            max_execution_phase_time_idx = group.EXECUTION_PHASE_TIME.idxmax()
            max_execution_phase_time_row = stats_frame.ix[max_execution_phase_time_idx]

            dist_margin = max_dist_row.TOTAL_DISTANCE - min_dist_row.TOTAL_DISTANCE
            run_time_margin = max_run_time_row.TOTAL_RUN_TIME - min_run_time_row.TOTAL_RUN_TIME
            execution_phase_time_margin = max_execution_phase_time_row.EXECUTION_PHASE_TIME - max_execution_phase_time_row.EXECUTION_PHASE_TIME

            print "team distance winner: {0}, minimax distance winner: {1}, run time winner: {2}, execution phase time winner: {3}".format(min_dist_row.MECHANISM, minimax_row.MECHANISM, min_run_time_row.MECHANISM, min_execution_phase_time_row.MECHANISM)
            print "team distance win margin: {0}, minimax distance margin: {1}, run time win margin: {2}, execution phase time margin: {3}".format(dist_margin, minimax_margin, run_time_margin, execution_phase_time_margin)
            print "team distance winner bag filename: {0}".format(min_dist_row.BAG_FILENAME)
            print "team distance loser bag filename: {0}".format(max_dist_row.BAG_FILENAME)
            print "run time winner bag filename: {0}".format(min_run_time_row.BAG_FILENAME)
            print "run time loser bag filename: {0}".format(max_run_time_row.BAG_FILENAME)
            print "execution phase time winner bag filename: {0}".format(min_execution_phase_time_row.BAG_FILENAME)
            print "execution phase time loser bag filename: {0}".format(max_execution_phase_time_row.BAG_FILENAME)
            print "minimax distance winner bag filename: {0}".format(minimax_row.BAG_FILENAME)
            print "minimax distance loser bag filename: {0}".format(minimax_loser_row.BAG_FILENAME)

            robot_graph = igraph.Graph()
            robot_graph.add_vertices(ROBOT_NAMES)

            # Turn on weighting
            robot_graph.es['weight'] = 1.0

            # For every pair of robots
            for pair in combinations(ROBOT_NAMES, 2):
                first_robot_name = pair[0]
                second_robot_name = pair[1]

                source_point = mrta.Point(min_dist_row["{0}_STARTX".format(first_robot_name.upper())],
                                          min_dist_row["{0}_STARTY".format(first_robot_name.upper())])
                source_pose = planner_proxy._point_to_pose(source_point)

                target_point = mrta.Point(min_dist_row["{0}_STARTX".format(second_robot_name.upper())],
                                          min_dist_row["{0}_STARTY".format(second_robot_name.upper())])
                target_pose = planner_proxy._point_to_pose(target_point)

                distance = planner_proxy.get_path_cost(source_pose, target_pose)

                robot_graph[first_robot_name, second_robot_name] = distance

            dist_matrix = robot_graph.get_adjacency(type=2, attribute='weight').data

            print("robot_graph: {0}, distances: {1}".format(robot_graph.summary(),
                                                            robot_graph.es['weight']))
            #print("Distance-weighted adjacency matrix: {0}".format(pp.pformat(dist_matrix)))

            robot_graph_mst = robot_graph.spanning_tree(weights=robot_graph.es['weight'])
            # print("robot_graph_mst: {0}, distances: {1}".format(robot_graph_mst.summary(),
            #                                                     robot_graph_mst.es['weight']))
            # print task_graph_mst
            team_diameter = robot_graph.diameter(directed=False, weights='weight')
            print "team diameter: {0}".format(team_diameter)

            robot_median_distances = [min_dist_row.ROBOT1_DISTANCE_TO_MEDIAN,
                                      min_dist_row.ROBOT2_DISTANCE_TO_MEDIAN,
                                      min_dist_row.ROBOT3_DISTANCE_TO_MEDIAN]

            print "robot median distances: {0}".format(robot_median_distances)

            min_median_dist = min(robot_median_distances)
            print "min_median_dist: {0}".format(min_median_dist)

            max_median_dist = max(robot_median_distances)
            print "max_median_dist: {0}".format(max_median_dist)

            median_spread = max_median_dist - min_median_dist
            print "median distance spread: {0}".format(median_spread)

            # Keep track of every robot's distance to every median
            # Key is robot id, value is a dict of median task_id => distance
            distance_to_all_medians = {}
            for robot_name in ROBOT_NAMES:
                distance_to_all_medians[robot_name] = {}

            # For each robot, count how many medians is is the "closest" to
            greedy_median_count = {}
            for robot_name in ROBOT_NAMES:
                greedy_median_count[robot_name] = 0

            median_task_ids = min_dist_row.MEDIAN_TASK_IDS.split('-')

            task_dict = target_point_configs[min_dist_row.TASK_FILE]

            for median_task_id in median_task_ids:
                min_robot_id = None
                min_robot_distance = None

                source_point = mrta.Point(task_dict[median_task_id][0],
                                          task_dict[median_task_id][1])
                source_pose = planner_proxy._point_to_pose(source_point)

                for robot_name in ROBOT_NAMES:
                    target_point = mrta.Point(min_dist_row["{0}_STARTX".format(robot_name.upper())],
                                              min_dist_row["{0}_STARTY".format(robot_name.upper())])
                    target_pose = planner_proxy._point_to_pose(target_point)

                    distance = planner_proxy.get_path_cost(source_pose, target_pose)

                    distance_to_all_medians[robot_name][median_task_id] = distance

                    if not min_robot_distance or distance < min_robot_distance:
                        min_robot_id = robot_name
                        min_robot_distance = distance

                greedy_median_count[min_robot_id] += 1

            median_assignment_spread = max(greedy_median_count.values()) - min(greedy_median_count.values())

            for row in (min_dist_row, min_run_time_row, min_execution_phase_time_row, minimax_row):
                row['MAX_DISTANCE_TO_MEDIAN'] = max_median_dist
                row['MIN_DISTANCE_TO_MEDIAN'] = min_median_dist
                row['MEDIAN_SPREAD'] = median_spread
                row['TEAM_DIAMETER'] = team_diameter
                row['MEDIAN_ASSIGNMENT_SPREAD'] = median_assignment_spread

                total_distance_to_all_medians = 0.0
                for robot_name in distance_to_all_medians:
                    robot_distance_to_all_medians = 0.0
                    for median_task_id in distance_to_all_medians[robot_name]:
                        robot_distance_to_all_medians += distance_to_all_medians[robot_name][median_task_id]

                    row["{0}_DISTANCE_TO_ALL_MEDIANS".format(robot_name.upper())] = robot_distance_to_all_medians

                    total_distance_to_all_medians += robot_distance_to_all_medians

                row['TOTAL_DISTANCE_TO_ALL_MEDIANS'] = total_distance_to_all_medians

            out_dist_csv.writerow([min_dist_row[f] for f in OUT_FIELDNAMES])
            out_run_time_csv.writerow([min_run_time_row[f] for f in OUT_FIELDNAMES])
            out_execution_phase_time_csv.writerow([min_execution_phase_time_row[f] for f in OUT_FIELDNAMES])
            out_minimax_csv.writerow([minimax_row[f] for f in OUT_FIELDNAMES])

            # for csv_file in (out_dist_csv, out_run_time_csv, out_execution_phase_time_csv, out_minimax_csv):
            #     csv_file.writerow([min_dist_row[f] for f in OUT_FIELDNAMES])

    except IOError:
        print "Something went wrong!"
        stop_ros()
        time.sleep(3)
        sys.exit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Produce training files from summary statistics.')

    parser.add_argument('-i', '--input',
                        default=IN_CSV_FILENAME,
                        help='The .csv file containing summary statistics,')

    parser.add_argument('-od', '--out_distance',
                        default=OUT_DIST_CSV_FILENAME,
                        help='The file to write for minimum distance-based training.')

    parser.add_argument('-ort', '--out_run_time',
                        default=OUT_RUN_TIME_CSV_FILENAME,
                        help='The file to write for minimum run time-based training.')

    parser.add_argument('-oet', '--out_execution_phase_time',
                        default=OUT_EXECUTION_PHSE_TIME_CSV_FILENAME,
                        help='The file to write for minimum execution phase time-based training.')

    parser.add_argument('-omi', '--out_minimax',
                        default=OUT_MINIMAX_CSV_FILENAME,
                        help='The file to write for minimax distance-based training.')

    parser.add_argument("--task_dir",
                        nargs='?',
                        default=TASK_FILES_DEFAULT,
                        help="Location of task configuration (target point) files")

    args = parser.parse_args()

    in_file = args.input
    out_dist = args.out_distance
    out_run_time = args.out_run_time
    out_execution_phase_time = args.out_execution_phase_time
    out_minimax = args.out_minimax
    task_dir = args.task_dir

    print(in_file, out_dist, out_run_time, out_execution_phase_time, out_minimax, task_dir)

    start_ros()
    write_training_files(in_file, out_dist, out_run_time, out_execution_phase_time, out_minimax, task_dir)
    stop_ros()
