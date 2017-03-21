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
from scipy.spatial import distance
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
import mrta.file_db
import mrta.msg
import mrta.mrta_planner_proxy

TASKS_DB_FILENAME = 'tasks.db'

IN_CSV_FILENAME = 'stats.csv'
OUT_DIST_CSV_FILENAME = 'team_distance.csv'
OUT_RUN_TIME_CSV_FILENAME = 'run_time.csv'
OUT_EXECUTION_PHSE_TIME_CSV_FILENAME = 'execution_phase_time.csv'
OUT_MINIMAX_CSV_FILENAME = 'minimax_distance.csv'

OUT_FIELDNAMES = ['MISSION_ID',
                  'TOTAL_DISTANCE_TO_ASSIGNED_MEDIANS',
                  'TOTAL_DISTANCE_TO_ALL_MEDIANS',
                  'MAX_DISTANCE_TO_ASSIGNED_MEDIAN',
                  'MAX_DISTANCE_TO_ANY_MEDIAN',
                  'MIN_DISTANCE_TO_ASSIGNED_MEDIAN',
                  'MIN_DISTANCE_TO_ANY_MEDIAN',
                  'ASSIGNED_MEDIAN_DISTANCE_SPREAD',
                  'TOTAL_MEDIAN_DISTANCE_SPREAD',
                  'TEAM_DIAMETER',
                  'AVERAGE_TEAMMATE_DISTANCE',
                  'AVERAGE_TEAM_CENTROID_DISTANCE',
                  'GREEDY_MEDIAN_COUNT_SPREAD',
                  'PSI_SPREAD',
                  'ROBOT1_DISTANCE_TO_ASSIGNED_MEDIAN',
                  'ROBOT1_DISTANCE_TO_ALL_MEDIANS',
                  'ROBOT1_STARTX',
                  'ROBOT1_STARTY',
                  'ROBOT2_DISTANCE_TO_ASSIGNED_MEDIAN',
                  'ROBOT2_DISTANCE_TO_ALL_MEDIANS',
                  'ROBOT2_STARTX',
                  'ROBOT2_STARTY',
                  'ROBOT3_DISTANCE_TO_ASSIGNED_MEDIAN',
                  'ROBOT3_DISTANCE_TO_ALL_MEDIANS',
                  'ROBOT3_STARTX',
                  'ROBOT3_STARTY',
                  'TASK_FILE',
                  'WINNER_DIFFERENCE',
                  'MAXIMUM_ROBOT_DISTANCE',
                  'EXECUTION_PHASE_TIME',
                  'TOTAL_DISTANCE',
                  'TOTAL_RUN_TIME',
                  'MECHANISM']

ROBOT_NAMES = ['robot1', 'robot2', 'robot3']

# Default location of task (target point) files
TASK_FILES_DEFAULT = "{0}/task_files".format(rospkg.RosPack().get_path('mrta_auctioneer'))

BAG_ROOT_DEFAULT = '/home/eric/nitsy76@gmail.com/research/MRTeAm/bags/chadwick'

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

# MAP_FILE = 'smartlab_ugv_arena_v2.png'
# WORLD_FILE = 'smartlab_ugv_arena_dummy_robot.world'

MAP_FILE = 'map-strand-first-floor-restricted-5cm.yaml'
WORLD_FILE = 'strand_restricted_dummy_robot.world'

# NOGUI_FLAG = ''
NOGUI_FLAG = '-g'

planner_proxy = None
main_proc = None


def find_file(name, path):
    for root, dirs, files in os.walk(path):
        if name in files:
            return os.path.join(root, name)


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
        if main_proc:
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


def read_point_config(task_db, scenario_id):
    global target_point_configs

    print "Reading points from {0}...".format(scenario_id)

    scenario = task_db[scenario_id]

    target_point_configs[scenario_id] = {}
    for task in scenario:
        target_point_configs[scenario_id][task.task_id] = (task.locaion.x, task.location.y)


def get_psi_spread(psi_bag):
    spread = 0

    robot_award_counts = defaultdict(int)

    try:
        for topic, msg, msg_time in psi_bag.read_messages('/tasks/award'):
            robot_award_counts[msg.robot_id] += len(msg.tasks)
    except:
        print(sys.exc_info()[0])
        return -1

    spread = max(robot_award_counts.values()) - min(robot_award_counts.values())

    return spread


def write_training_files(in_file, out_dist, out_run_time, out_execution_phase_time, out_minimax, task_dir, bag_root):
    global planner_proxy, target_point_configs
    try:

        stats_frame = pd.read_csv(in_file)

        start_ros()

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

        # for scenario_id, group in stats_frame.groupby([stats_frame.ROBOT1_STARTX, stats_frame.ROBOT1_STARTY]):

        # Group by start positions
        # for scenario_id, group in psi_ssi.groupby([stats_frame.ROBOT1_STARTX, stats_frame.ROBOT1_STARTY]):

        task_db = None
        try:
            # Save the poses in the tasks database
            task_db = mrta.file_db.FileDB(TASKS_DB_FILENAME)
        except IOError:
            print "Couldn't read tasks from {0}! Exiting.".format(TASKS_DB_FILENAME)
            sys.exit(1)

        # Group by task file
        for scenario_id, group in psi_ssi.groupby([stats_frame.SCENARIO_ID]):

            print scenario_id

            # # Only record a training instance if runs for all three mechanisms (PSI, SSI, OSI) succeeded
            # if group.ROBOT1_STARTX.count() < 3:
            #     continue

            # Only record a training instance if runs for both mechanisms (PSI, SSI) succeeded
            if group.ROBOT1_STARTX.count() < 2:
                continue

            if scenario_id not in target_point_configs:
                read_point_config(task_db, scenario_id)

            # We only have two rows in this group (one per PSI and SSI)
            first_row = group.iloc[0]
            second_row = group.iloc[1]

            # Find the PSI row/run
            if first_row.MECHANISM == 'PSI':
                psi_row = first_row
            elif second_row.MECHANISM == 'PSI':
                psi_row = second_row
            else:
                print("Couldn't identify PSI run! Skipping...")
                continue

            # Find the PSI bag filename
            psi_bag_filename = psi_row.BAG_FILENAME

            psi_bag_filepath = find_file(psi_bag_filename, bag_root)

            if not psi_bag_filepath:
                print("Couldn't find {0} in {1}! Skipping...".format(psi_bag_filename, psi_bag_filepath))
                continue

            try:
                print("Opening PSI bag at {0}...".format(psi_bag_filepath))
                psi_bag = rosbag.Bag(psi_bag_filepath)
            except:
                print(sys.exc_info()[0])
                print("Couldn't open {0}! Skipping...".format(psi_bag_filepath))
                continue

            # Examine the bag to determine how imbalanced the PSI allocation was
            psi_spread = get_psi_spread(psi_bag)
            psi_bag.close()

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

            minimax_margin = abs(first_row_max_dist - second_row_max_dist)
            team_distance_margin = max_dist_row.TOTAL_DISTANCE - min_dist_row.TOTAL_DISTANCE
            run_time_margin = max_run_time_row.TOTAL_RUN_TIME - min_run_time_row.TOTAL_RUN_TIME
            execution_phase_time_margin = max_execution_phase_time_row.EXECUTION_PHASE_TIME - min_execution_phase_time_row.EXECUTION_PHASE_TIME

            minimax_row['WINNER_DIFFERENCE'] = minimax_margin
            min_dist_row['WINNER_DIFFERENCE'] = team_distance_margin
            min_run_time_row['WINNER_DIFFERENCE'] = run_time_margin
            min_execution_phase_time_row['WINNER_DIFFERENCE'] = execution_phase_time_margin

            print "team distance winner: {0}, minimax distance winner: {1}, run time winner: {2}, execution phase time winner: {3}".format(min_dist_row.MECHANISM, minimax_row.MECHANISM, min_run_time_row.MECHANISM, min_execution_phase_time_row.MECHANISM)
            print "team distance win margin: {0}, minimax distance margin: {1}, run time win margin: {2}, execution phase time margin: {3}".format(team_distance_margin, minimax_margin, run_time_margin, execution_phase_time_margin)
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

                teammate_distance = planner_proxy.get_path_cost(source_pose, target_pose)

                robot_graph[first_robot_name, second_robot_name] = teammate_distance

            dist_matrix = robot_graph.get_adjacency(type=2, attribute='weight').data

            print("robot_graph: {0}, distances: {1}".format(robot_graph.summary(),
                                                            robot_graph.es['weight']))
            # print("Distance-weighted adjacency matrix: {0}".format(pp.pformat(dist_matrix)))

            # robot_graph_mst = robot_graph.spanning_tree(weights=robot_graph.es['weight'])
            # print("robot_graph_mst: {0}, distances: {1}".format(robot_graph_mst.summary(),
            #                                                     robot_graph_mst.es['weight']))
            # print task_graph_mst
            team_diameter = robot_graph.diameter(directed=False, weights='weight')
            print "team diameter: {0}".format(team_diameter)

            average_teammate_distance = np.mean(robot_graph.es['weight'])
            print "average teammate distance: {0}".format(average_teammate_distance)

            # Find the euclidean center of the team, then measure the average team
            # member distance to that center.
            team_centroid_x = sum([min_dist_row["{0}_STARTX".format(rn.upper())] for rn in ROBOT_NAMES]) * 1. / len(ROBOT_NAMES)
            team_centroid_y = sum([min_dist_row["{0}_STARTY".format(rn.upper())] for rn in ROBOT_NAMES]) * 1. / len(ROBOT_NAMES)

            total_team_centroid_distance = 0.0
            for robot_name in ROBOT_NAMES:
                robot_x = min_dist_row["{0}_STARTX".format(robot_name.upper())]
                robot_y = min_dist_row["{0}_STARTY".format(robot_name.upper())]
                total_team_centroid_distance += distance.euclidean((robot_x, robot_y), (team_centroid_x, team_centroid_y))

            average_team_centroid_distance = total_team_centroid_distance * 1. / len(ROBOT_NAMES)

            print "average team centroid distance: {0}".format(average_team_centroid_distance)

            assigned_robot_median_distances = [min_dist_row.ROBOT1_DISTANCE_TO_ASSIGNED_MEDIAN,
                                               min_dist_row.ROBOT2_DISTANCE_TO_ASSIGNED_MEDIAN,
                                               min_dist_row.ROBOT3_DISTANCE_TO_ASSIGNED_MEDIAN]

            print "robot median distances: {0}".format(assigned_robot_median_distances)

            min_assigned_median_dist = min(assigned_robot_median_distances)
            print "min_assigned_median_dist: {0}".format(min_assigned_median_dist)

            max_assigned_median_dist = max(assigned_robot_median_distances)
            print "max_assigned_median_dist: {0}".format(max_assigned_median_dist)

            assigned_median_distance_spread = max_assigned_median_dist - min_assigned_median_dist
            print "assigned median distance spread: {0}".format(assigned_median_distance_spread)

            # Keep track of every robot's distance to every median
            # Key is robot id, value is a dict of median task_id => distance
            distance_to_all_medians = {}
            for robot_name in ROBOT_NAMES:
                distance_to_all_medians[robot_name] = {}

            # For each robot, count how many medians it is the "closest" to
            greedy_median_count = {}
            for robot_name in ROBOT_NAMES:
                greedy_median_count[robot_name] = 0

            median_task_ids = min_dist_row.MEDIAN_TASK_IDS.split('-')

            task_dict = target_point_configs[min_dist_row.SCENARIO_ID]

            for median_task_id in median_task_ids:
                min_robot_id = None
                min_robot_distance = None

                source_point = mrta.Point(task_dict[median_task_id][0],   # x
                                          task_dict[median_task_id][1])   # y
                source_pose = planner_proxy._point_to_pose(source_point)

                for robot_name in ROBOT_NAMES:
                    target_point = mrta.Point(min_dist_row["{0}_STARTX".format(robot_name.upper())],
                                              min_dist_row["{0}_STARTY".format(robot_name.upper())])
                    target_pose = planner_proxy._point_to_pose(target_point)

                    median_distance = planner_proxy.get_path_cost(source_pose, target_pose)

                    distance_to_all_medians[robot_name][median_task_id] = median_distance

                    if not min_robot_distance or median_distance < min_robot_distance:
                        min_robot_id = robot_name
                        min_robot_distance = median_distance

                greedy_median_count[min_robot_id] += 1

            greedy_median_count_spread = max(greedy_median_count.values()) - min(greedy_median_count.values())

            for row in (min_dist_row, min_run_time_row, min_execution_phase_time_row, minimax_row):
                row['TOTAL_DISTANCE_TO_ASSIGNED_MEDIANS'] = sum(assigned_robot_median_distances)
                row['MAX_DISTANCE_TO_ASSIGNED_MEDIAN'] = max_assigned_median_dist
                row['MIN_DISTANCE_TO_ASSIGNED_MEDIAN'] = min_assigned_median_dist
                row['ASSIGNED_MEDIAN_DISTANCE_SPREAD'] = assigned_median_distance_spread
                row['TEAM_DIAMETER'] = team_diameter
                row['AVERAGE_TEAMMATE_DISTANCE'] = average_teammate_distance
                row['AVERAGE_TEAM_CENTROID_DISTANCE'] = average_team_centroid_distance
                row['GREEDY_MEDIAN_COUNT_SPREAD'] = greedy_median_count_spread

                row['PSI_SPREAD'] = psi_spread

                total_distance_to_all_medians = 0.0

                max_distance_to_any_median = None
                min_distance_to_any_median = None

                for robot_name in distance_to_all_medians:
                    robot_distance_to_all_medians = 0.0
                    for median_task_id in distance_to_all_medians[robot_name]:
                        robot_median_distance = distance_to_all_medians[robot_name][median_task_id]

                        robot_distance_to_all_medians += robot_median_distance

                        if not max_distance_to_any_median or robot_median_distance > max_distance_to_any_median:
                            max_distance_to_any_median = robot_median_distance

                        if not min_distance_to_any_median or robot_median_distance < min_distance_to_any_median:
                            min_distance_to_any_median = robot_median_distance

                    row["{0}_DISTANCE_TO_ALL_MEDIANS".format(robot_name.upper())] = robot_distance_to_all_medians

                    total_distance_to_all_medians += robot_distance_to_all_medians

                row['TOTAL_DISTANCE_TO_ALL_MEDIANS'] = total_distance_to_all_medians
                row['MAX_DISTANCE_TO_ANY_MEDIAN'] = max_distance_to_any_median
                row['MIN_DISTANCE_TO_ANY_MEDIAN'] = min_distance_to_any_median
                row['TOTAL_MEDIAN_DISTANCE_SPREAD'] = max_distance_to_any_median - min_distance_to_any_median

                row['MAXIMUM_ROBOT_DISTANCE'] = max(row['ROBOT1_DISTANCE'],
                                                    row['ROBOT2_DISTANCE'],
                                                    row['ROBOT3_DISTANCE'])

                row['SCENARIO_ID'] = scenario_id

            out_dist_csv.writerow([min_dist_row[f] for f in OUT_FIELDNAMES])
            out_run_time_csv.writerow([min_run_time_row[f] for f in OUT_FIELDNAMES])
            out_execution_phase_time_csv.writerow([min_execution_phase_time_row[f] for f in OUT_FIELDNAMES])
            out_minimax_csv.writerow([minimax_row[f] for f in OUT_FIELDNAMES])

            # for csv_file in (out_dist_csv, out_run_time_csv, out_execution_phase_time_csv, out_minimax_csv):
            #     csv_file.writerow([min_dist_row[f] for f in OUT_FIELDNAMES])

    except IOError:
        print "Something went wrong!"
    finally:
        stop_ros()


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

    parser.add_argument('-br', '--bag_root',
                        default=BAG_ROOT_DEFAULT,
                        help="Root directory of bag files.")

    args = parser.parse_args()

    in_file = args.input
    out_dist = args.out_distance
    out_run_time = args.out_run_time
    out_execution_phase_time = args.out_execution_phase_time
    out_minimax = args.out_minimax
    task_dir = args.task_dir
    bag_root = args.bag_root

    print(in_file, out_dist, out_run_time, out_execution_phase_time, out_minimax, task_dir, bag_root)

    write_training_files(in_file, out_dist, out_run_time, out_execution_phase_time, out_minimax, task_dir, bag_root)
