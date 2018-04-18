#!/usr/bin/env python

import argparse
import glob
import os.path
import sys
import yaml

import mrta
import mrta.file_db

TASKS_DB_FILENAME = 'tasks.db'


def convert_task_files(task_dir, force=False):

    task_db = mrta.file_db.FileDB(TASKS_DB_FILENAME)

    for task_file in glob.glob(os.path.join(task_dir, '*.yaml')):
        # print("task filename: {0}".format(task_file))

        # Get the filename and remove the '.yaml' extension
        scenario_name = os.path.splitext(os.path.basename(task_file))[0]

        # print("scenario_name: {0}".format(scenario_name))

        if not task_db.exists(scenario_name) or force:
            print("Writing {0} to task database...".format(scenario_name))
            try:
                task_file = open(task_file, 'rb')
                task_list = []

                yaml_tasks = yaml.load(task_file)
                for yaml_task in yaml_tasks:
                    # task_id is string-ified here because an id may one day be
                    # an MD5 hash or some other non-integer value. They just happen
                    # to be integers here.
                    new_task = mrta.SensorSweepTask(str(yaml_task['task_id']),
                                                    float(yaml_task['location']['x']),
                                                    float(yaml_task['location']['y']),
                                                    0.0,  # z
                                                    int(yaml_task['num_robots']),
                                                    float(yaml_task['duration']),
                                                    yaml_task['depends'])
                    task_list.append(new_task)

                task_db[scenario_name] = task_list

            except IOError:
                print("Couldn't open {0} for reading!".format(task_file))

    task_db.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert/import (yaml) file-based into a file-based database.')

    parser.add_argument('task_dir',
                        help='The directory containing task file to convert.')

    parser.add_argument('-f', '--force',
                        action='store_true',
                        help='Force saving the task to the database (overwrite).')

    args = parser.parse_args()

    task_dir = args.task_dir
    force = args.force

    sys.exit(convert_task_files(task_dir, force))
