#!/usr/bin/env python

import argparse
from collections import defaultdict
import glob
import os
from os import listdir
import os.path
from os.path import isdir, isfile, join
import re
import rosbag
import rospkg
import shutil
import sys
import yaml

import mrta.msg


def check_complete(bag_paths):
    dt_re = re.compile('(.*)\.bag')

    num_failed = 0
    num_succeeded = 0

    for i, bag_path in enumerate(bag_paths):
        # print("Reading {0}".format(bag_path))

        bag = None
        try:
            bag = rosbag.Bag(bag_path)
        except:
            print("Couldn't open {0} for reading!".format(bag_path))
            continue

        bag_filename = os.path.basename(bag_path)
        bag_dirname = os.path.dirname(bag_path)

        # # Create the 'failed' directory if it doesn't exist
        # failed_dirname = os.path.join(bag_dirname, 'FAILED')
        # if not os.path.isdir(failed_dirname):
        #     print('...Creating {0}'.format(failed_dirname))
        #     os.mkdir(failed_dirname)

        (map, start_config, mechanism, task_file, remainder) = bag_filename.split('__')

        # Date/time
        dt_match = dt_re.search(remainder)
        dt_string = dt_match.group(1) # 'DATETIME'

        run_msgs = defaultdict(list)

        for topic, msg, msg_time in bag.read_messages():
            run_msgs[topic].append(msg)

        bag.close()

        exp_msgs = {}
        for msg in run_msgs['/experiment']:
            # print msg
            exp_msgs[msg.event] = msg

        print('{0} {1} ({2}):'.format(task_file, mechanism, dt_string)),

        if mrta.msg.ExperimentEvent.END_EXPERIMENT in exp_msgs:
            print('  COMPLETE!')
            num_succeeded += 1
        else:
            print('  FAILED!')
            num_failed += 1
            # move the file to the 'failed' directory
            # shutil.move(bag_path, os.path.join(failed_dirname, bag_filename))

    print("Total: {0}, (succeeded: {1}, failed: {2})".format(num_failed+num_succeeded, num_succeeded, num_failed))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Check if the experiments for a given set of bag paths actually completed.')

    parser.add_argument('bag_file',
                        nargs='+',
                        help='Path to the bag file of the experiment to plot.')

    args = parser.parse_args()

    bag_files = args.bag_file

    # pp.pprint(bag_files)

    # Make one, flat list of paths to log files
    bag_paths = []

    for path_arg in bag_files:
        bag_paths.extend(glob.glob(path_arg))

    # print("bag paths: ")
    # pp.pprint(bag_paths)

    check_complete(bag_paths)
