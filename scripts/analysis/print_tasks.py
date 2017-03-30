#!/usr/bin/env python

import argparse
import cairo
from collections import defaultdict
import glob
from itertools import combinations
import math
import mrta
import mrta.file_db
import mrta.msg
import os
import os.path
import re
import rosbag
import rospkg
import rospy
import signal
import subprocess
import sys
import time
import traceback
import yaml

import geometry_msgs.msg
import mrta.mrta_planner_proxy

# For debugging
import pprint
pp = pprint.PrettyPrinter(indent=4)

TASKS_DB_FILENAME = 'tasks.db'


def print_tasks(scenario_id):

    task_db = None
    try:
        # Save the poses in the tasks database
        task_db = mrta.file_db.FileDB(TASKS_DB_FILENAME)
    except IOError:
        print "Couldn't read tasks from {0}! Exiting.".format(TASKS_DB_FILENAME)
        sys.exit(1)

    print "Reading points from {0}...".format(scenario_id)

    scenario = task_db[scenario_id]

    pp.pprint(scenario)

    # target_point_configs[scenario_id] = {}
    # for task in scenario:
    #     target_point_configs[scenario_id][task.task_id] = (task.location.x, task.location.y)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot the task graph for the given experiments (bags).')

    parser.add_argument('scenario_id',
                        help='Scenario to print.')

    args = parser.parse_args()

    scenario_id = args.scenario_id

    print_tasks(scenario_id)
