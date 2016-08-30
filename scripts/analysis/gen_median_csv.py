#!/usr/bin/env python

import argparse
from collections import defaultdict
import csv
import glob
from itertools import chain, combinations
import math
import numpy as np
import os
import pandas as pd
import pprint
import re
import signal
import subprocess
import sys
import time

import igraph

import rospy
import rosbag

import mrta
import mrta.msg
import mrta.mrta_planner_proxy


IN_CSV_FILENAME = 'stats.csv'
OUT_DIST_CSV_FILENAME = 'median_distance.csv'
OUT_TIME_CSV_FILENAME = 'median_time.csv'
OUT_MINIMAX_CSV_FILENAME = 'median_minimax_distance.csv'

OUT_FIELDNAMES = ['TOTAL_DISTANCE_TO_MEDIANS',
                  'MEDIAN_SPREAD',
                  'TEAM_DIAMETER',
                  'ROBOT1_DISTANCE_TO_MEDIAN',
                  'ROBOT1_STARTX',
                  'ROBOT1_STARTY',
                  'ROBOT2_DISTANCE_TO_MEDIAN',
                  'ROBOT2_STARTX',
                  'ROBOT2_STARTY',
                  'ROBOT3_DISTANCE_TO_MEDIAN',
                  'ROBOT3_STARTX',
                  'ROBOT3_STARTY',
                  'MECHANISM']

ROBOT_NAMES = ['robot1', 'robot2', 'robot3']

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


def write_training_files(in_file, out_dist, out_time, out_minimax):
    global planner_proxy
    try:

        stats_frame = pd.read_csv(in_file)

        # Only look at PSI and SSI for now
        psi_ssi = stats_frame[stats_frame.MECHANISM.isin(['PSI','SSI'])]

        out_dist_csv = csv.writer(open(out_dist, 'wb'))
        out_dist_csv.writerow(OUT_FIELDNAMES)

        out_time_csv = csv.writer(open(out_time, 'wb'))
        out_time_csv.writerow(OUT_FIELDNAMES)

        out_minimax_csv = csv.writer(open(out_minimax, 'wb'))
        out_minimax_csv.writerow(OUT_FIELDNAMES)

        # for name, group in stats_frame.groupby([stats_frame.ROBOT1_STARTX, stats_frame.ROBOT1_STARTY]):
        for name, group in psi_ssi.groupby([stats_frame.ROBOT1_STARTX, stats_frame.ROBOT1_STARTY]):

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

            min_time_idx = group.TOTAL_MOVEMENT_TIME.idxmin()
            min_time_row = stats_frame.ix[min_time_idx]
            max_time_idx = group.TOTAL_MOVEMENT_TIME.idxmax()
            max_time_row = stats_frame.ix[max_time_idx]

            dist_margin = max_dist_row.TOTAL_DISTANCE - min_dist_row.TOTAL_DISTANCE
            time_margin = max_time_row.TOTAL_MOVEMENT_TIME - min_time_row.TOTAL_MOVEMENT_TIME

            print "distance winner: {0}, minimax distance winner: {1}, time winner: {2}".format(min_dist_row.MECHANISM, minimax_row.MECHANISM, min_time_row.MECHANISM)
            print "distance win margin: {0}, time win margin: {1}, minimax distance margin: {2}".format(dist_margin, time_margin, minimax_margin)
            print "distance winner bag filename: {0}".format(min_dist_row.BAG_FILENAME)
            print "distance loser bag filename: {0}".format(max_dist_row.BAG_FILENAME)
            print "time winner bag filename: {0}".format(min_time_row.BAG_FILENAME)
            print "time loser bag filename: {0}".format(max_time_row.BAG_FILENAME)
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

            min_dist_row['MEDIAN_SPREAD'] = median_spread
            min_time_row['MEDIAN_SPREAD'] = median_spread
            minimax_row['MEDIAN_SPREAD'] = median_spread

            min_dist_row['TEAM_DIAMETER'] = team_diameter
            min_time_row['TEAM_DIAMETER'] = team_diameter
            minimax_row['TEAM_DIAMETER'] = team_diameter

            out_dist_csv.writerow([min_dist_row[f] for f in OUT_FIELDNAMES])
            out_time_csv.writerow([min_time_row[f] for f in OUT_FIELDNAMES])
            out_minimax_csv.writerow([minimax_row[f] for f in OUT_FIELDNAMES])

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

    parser.add_argument('-ot', '--out_time',
                        default=OUT_TIME_CSV_FILENAME,
                        help='The file to write for minimum time-based training.')

    parser.add_argument('-om', '--out_minimax',
                        default=OUT_MINIMAX_CSV_FILENAME,
                        help='The file to write for minimax distance-based training.')

    args = parser.parse_args()

    in_file = args.input
    out_dist = args.out_distance
    out_time = args.out_time
    out_minimax = args.out_minimax

    print(in_file, out_dist, out_time, out_minimax)

    start_ros()
    write_training_files(in_file, out_dist, out_time, out_minimax)
    stop_ros()
