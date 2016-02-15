#!/usr/bin/env python

import argparse
import rospy
import time
import yaml

import mrta.msg

import pprint

pp = pprint.PrettyPrinter(indent=4)


CONFIG_FILE_DEFAULT = 'reset_positions.yaml'
award_pub = None
experiment_pub = None


def stamp(msg):
    """ Set the timestamp of a message to the current wall-clock time."""
    rospy.rostime.switch_to_wallclock()
    msg.header.stamp = rospy.rostime.get_rostime()


def init_node():
    global award_pub, experiment_pub

    rospy.init_node('reset_positions')

    award_pub = rospy.Publisher('/tasks/award',
                                mrta.msg.TaskAward,
                                queue_size=3)

    experiment_pub = rospy.Publisher('/experiment',
                                     mrta.msg.ExperimentEvent,
                                     queue_size=3)

    time.sleep(3)


def reset_positions(config_filename, start_config):
    global award_pub, experiment_pub

    init_node()

    time.sleep(3)

    # Send a message to mark the end of the experiment
    end_exp_msg = mrta.msg.ExperimentEvent()
    end_exp_msg.experiment_id = 'reset'
    end_exp_msg.event = mrta.msg.ExperimentEvent.END_EXPERIMENT
    stamp(end_exp_msg)
    experiment_pub.publish(end_exp_msg)

    time.sleep(3)

    config_file = open(config_filename, 'rb')
    config = yaml.load(config_file)

    start_positions = config[start_config]

    for robot_name in sorted(start_positions):

        robot = start_positions[robot_name]

        award_msg = mrta.msg.TaskAward()
        award_msg.robot_id = robot_name

        task_msg = mrta.msg.SensorSweepTask()
        task_msg.task.task_id = 'reset'
        task_msg.task.depends = []
        task_msg.task.type = 'SENSOR_SWEEP'
        task_msg.task.num_robots = 1
        task_msg.task.duration = 0
        task_msg.location.x = robot['x']
        task_msg.location.y = robot['y']
        task_msg.location.z = 0

        award_msg.tasks.append(task_msg)

        stamp(award_msg)
        award_pub.publish(award_msg)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Send robots back to their start positions")

    parser.add_argument("config",
                        choices=['clustered', 'distributed'],
                        help="Robot starting configuration.")

    parser.add_argument("-f", "--filename",
                        help="Configuration filename",
                        default=CONFIG_FILE_DEFAULT)

    args = parser.parse_args()

    reset_positions(args.filename, args.config)
