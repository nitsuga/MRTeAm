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

import rosbag
import rospy
import tf.transformations

import mrta
import mrta.msg

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


class Robot(object):
    def __init__(self):
        self.name = None
        self.startx = None
        self.starty = None
        self.starta = None
        self.startd = None
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


def print_start_poses(bag_paths):

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

        print("bag_filename: {0}".format(bag_filename))
        (map, start_config, mechanism, task_file, remainder) = bag_filename.split('__')

        run_msgs = defaultdict(list)

        try:
            for topic, msg, msg_time in bag.read_messages():
                # msg.header.stamp = msg_time
                run_msgs[topic].append(msg)
        except:
            print("Couldn't read messages from {0}!".format(bag_path))
            print(sys.exc_info()[0])
            continue

        robots = {}
        for r_name in ROBOT_NAMES:
            robot = Robot()
            robot.name = r_name
            robots[r_name] = robot

        # per-robot stats
        for r_name in ROBOT_NAMES:
            robot = robots[r_name]

            for amcl_msg in run_msgs['/{0}/amcl_pose'.format(r_name)]:

                amcl_pose = amcl_msg.pose.pose

                if amcl_pose.position.x == 0 and amcl_pose.position.y == 0:
                    continue

                if not robot.startx:
                    robot.startx = amcl_pose.position.x
                    robot.starty = amcl_pose.position.y

                    my_q = (amcl_pose.orientation.x,
                            amcl_pose.orientation.y,
                            amcl_pose.orientation.z,
                            amcl_pose.orientation.w)
                    euler = tf.transformations.euler_from_quaternion(my_q)
                    robot.starta = euler[2]
                    robot.startd = robot.starta * 180. / math.pi

                    print r_name
                    print " X: {0}".format(robot.startx)
                    print " Y: {0}".format(robot.starty)
                    print " A: {0}".format(robot.startd)

                    break


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract summary statistics for the runs recorded in given bag files.')

    parser.add_argument('bag_file',
                        nargs='+',
                        help='Path to the bag file(s) of the experiment(s) to plot.')

    args = parser.parse_args()

    bag_files = args.bag_file

    # pp.pprint(bag_files)

    # Make one, flat list of paths to log files
    bag_paths = []

    for path_arg in bag_files:
        bag_paths.extend(glob.glob(path_arg))
    # print("bag paths: ")
    # pp.pprint(bag_paths)

    print_start_poses(bag_paths)
