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

# smartlab arena
# IMG_WIDTH, IMG_HEIGHT = 800, 600

# strand map
# IMG_WIDTH, IMG_HEIGHT = 376, 868
IMG_WIDTH, IMG_HEIGHT = 1880, 4338

# Every pixel of our '5cm' map represents 6.65cm. To convert from ROS coordinates
# to pixels we need the inverse of that
# SCALING_FACTOR = 0.15037594

# The inverse of 1.33
SCALING_FACTOR = 0.751879699

# Default location of task (target point) files
TASK_FILES_DEFAULT = "{0}/task_files".format(rospkg.RosPack().get_path('mrta_auctioneer'))

stroke_colors = {'robot_1': (1.0, 0.0, 0.0),  # Red
                 'robot_2': (0.0, 1.0, 0.0),  # Green
                 'robot_3': (0.0, 0.0, 1.0)}  # Blue

target_point_configs = {}

robot_names = ['robot_1', 'robot_2', 'robot_3']

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
        if main_proc:
            main_proc.send_signal(signal.SIGINT)
            main_proc.wait()
    except rospy.exceptions.ROSException as e:
        print(e)
        print("Error: {0}".format(sys.exc_info()[0]))


def _read_points(task_file):
    points = []

    # YAML config file
    if task_file.name.endswith('yaml'):
        yaml_tasks = yaml.load(task_file)

        for yaml_task in yaml_tasks:
            task_loc = yaml_task['location']
            x, y = float(task_loc['x']) * 100., float(task_loc['y']) * 100.
            points.append((yaml_task['task_id'], x, y))

    # .txt config file
    elif task_file.name.endswith('txt'):
        for line in task_file:
            # Ignore comments
            if line.startswith('#'):
                continue

            # Each line is "s x y n d" where
            # s   = seconds after start that the task appears
            # x,y = target point location
            # n   = number of robots required
            # d   = duration of the task
            task_fields = line.split()
            x, y = task_fields[1], task_fields[2]
            points.append([float(x) * 100., float(y) * 100.])

    return points


def read_point_config(task_db, scenario_id):
    global target_point_configs

    print "Reading points from {0}...".format(scenario_id)

    scenario = task_db[scenario_id]

    target_point_configs[scenario_id] = {}
    for task in scenario:
        target_point_configs[scenario_id][task.task_id] = (task.location.x, task.location.y)


def draw_target_points(ctx, scenario_id, run_msgs):
    global target_point_configs

    text_alpha = 1.0

    # Font style for printing points
    ctx.select_font_face('Sans', cairo.FONT_SLANT_NORMAL,
                         cairo.FONT_WEIGHT_BOLD)
    ctx.set_font_size(16. / IMG_WIDTH)

    ctx.set_line_width((1. / IMG_WIDTH))

    ctx.set_source_rgba(0, 0, 0, text_alpha)

    median_task_ids = []

    median_task_ids_re = re.compile('median task ids: \[(.*)\]')

    # Get the ids of the p-median tasks
    for debug_msg in run_msgs['/debug']:
        if debug_msg.key == 'auctioneer-median-ids':
            matches = re.search(median_task_ids_re, debug_msg.value)
            if matches:
                task_ids_string = matches.group(1)
                median_task_ids.extend(task_ids_string.split(','))
                break

    print "median task ids: {0}".format(median_task_ids)

    target_points = target_point_configs[scenario_id]

    for i, task_id in enumerate(target_points):

        target_point = target_point_configs[scenario_id][task_id]

        print("target_point: {0}".format(pp.pformat(target_point)))

        # task_id = target_point[0]
        task_x = target_point[0] * SCALING_FACTOR * 100.
        task_y = target_point[1] * SCALING_FACTOR * 100.

        print "plotting task {0} ({1}, {2})".format(task_id, task_x, task_y)

        #            print "Drawing task point at (%f,%f)" % (task_x, task_y)

        # Draw point number (i) as a label slightly above and to the right of the point
        ctx.move_to((task_x + 6) / IMG_WIDTH, (task_y + 6) / IMG_HEIGHT)
        ctx.transform(cairo.Matrix(yy=-1, y0=IMG_HEIGHT))
        ctx.show_text(str(task_id))
        ctx.stroke()
        ctx.transform(cairo.Matrix(yy=-1, y0=IMG_HEIGHT))

        # Draw p-medians as circles
        # if i == 2 or i == 11 or i == 13:
        if task_id in median_task_ids:
            print "task {0} is a median".format(task_id)
            ctx.set_source_rgb(.36, .36, 1.0)
            ctx.set_line_width((3. / IMG_WIDTH))
            ctx.arc(task_x / IMG_WIDTH, task_y / IMG_HEIGHT, 8./IMG_WIDTH, 0, 2*math.pi)
            ctx.stroke()
            ctx.fill()
        else:
            ctx.set_source_rgb(0, 0, 0)
            ctx.set_line_width((1. / IMG_WIDTH))

        ctx.move_to((task_x - 10) / IMG_WIDTH, (task_y - 10) / IMG_HEIGHT)
        ctx.line_to((task_x + 10) / IMG_WIDTH, (task_y + 10) / IMG_HEIGHT)
        ctx.stroke()

        ctx.move_to((task_x - 10) / IMG_WIDTH, (task_y + 10) / IMG_HEIGHT)
        ctx.line_to((task_x + 10) / IMG_WIDTH, (task_y - 10) / IMG_HEIGHT)
        ctx.stroke()

        ctx.set_source_rgb(0, 0, 0)


def _pose_equal(pose1, pose2):
    """ True if pose1 is a different position or orientation than pose2
    :param pose1:
    :param pose2:
    :return:
    """
    p1_pos = pose1.pose.pose.position
    p1_orient = pose1.pose.pose.orientation
    p2_pos = pose2.pose.pose.position
    p2_orient = pose2.pose.pose.orientation

    if p1_pos.x != p2_pos.x or p1_pos.y != p2_pos.y or p1_orient.z != p2_orient.z or p1_orient.w != p2_orient.w:
        return False

    return True


def draw_edges(ctx, scenario_id):
    global planner_proxy, target_point_configs

    line_alpha = 0.3

    # Trajectory line width
    ctx.set_line_width((1. / IMG_WIDTH))

    # print('alpha: {0}'.format(alpha))
    ctx.set_source_rgba(0, 0, 0, line_alpha)

    target_points = target_point_configs[scenario_id]

    # Iterate through every pair of target_points
    for pair in combinations(target_points, 2):
        # The task ids of the source and target
        source = target_point_configs[scenario_id][pair[0]]
        target = target_point_configs[scenario_id][pair[1]]

        # print("source: {0}, target: {1}".format(pp.pformat(source), pp.pformat(target)))

        source_pose = planner_proxy._point_to_pose(geometry_msgs.msg.Point(source[0], source[1], 0.0))
        target_pose = planner_proxy._point_to_pose(geometry_msgs.msg.Point(target[0], target[1], 0.0))

        # A plan is a list of poses (geometry_msgs.msg.Pose[])
        # print("source_pose: {0}, target_pose: {1}".format(pp.pformat(source_pose), pp.pformat(target_pose)))
        plan = planner_proxy._make_nav_plan(source_pose, target_pose)

        start_pose = None
        last_pose = None

        for plan_pose in plan:

            # print("plan_pose: {0}".format(pp.pformat(plan_pose)))

            pose_x = float(plan_pose.pose.position.x)
            pose_y = float(plan_pose.pose.position.y)

            if pose_x == 0 and pose_y == 0:
                continue

            if last_pose is None:
                ctx.move_to(pose_x * SCALING_FACTOR * 100. / IMG_WIDTH,
                            pose_y * SCALING_FACTOR * 100. / IMG_HEIGHT)
                # print "{0} start_pose: {1}:".format(r_name, start_pose)
            else:
                ctx.move_to(last_pose.pose.position.x * SCALING_FACTOR * 100. / IMG_WIDTH,
                            last_pose.pose.position.y * SCALING_FACTOR * 100. / IMG_HEIGHT)

            ctx.line_to(pose_x * SCALING_FACTOR * 100. / IMG_WIDTH,
                        pose_y * SCALING_FACTOR * 100. / IMG_HEIGHT)

            ctx.stroke()

            last_pose = plan_pose


def plot_task_graph(bag_paths, task_dir, map_file):
    global target_point_configs

    # Start the planner proxy
    start_ros()

    # Create the './trajectories' directory if it doesn't already exist
    traj_dir = './task_graphs'
    if not os.path.exists(traj_dir):
        os.makedirs(traj_dir)

    task_db = None
    try:
        # Save the poses in the tasks database
        task_db = mrta.file_db.FileDB(TASKS_DB_FILENAME)
    except IOError:
        print "Couldn't read tasks from {0}! Exiting.".format(TASKS_DB_FILENAME)
        sys.exit(1)

    for bag_path in bag_paths:
        print("Reading {0}".format(bag_path))

        bag = None
        try:
            bag = rosbag.Bag(bag_path)
        except:
            print("Couldn't open {0} for reading!".format(bag_path))
            print(sys.exc_info()[:2])
            continue

        run_msgs = defaultdict(list)
        try:
            for topic, msg, msg_time in bag.read_messages():
                # msg.header.stamp = msg_time
                run_msgs[topic].append(msg)
        except:
            print("Couldn't read messages from {0}!".format(bag_path))
            print(sys.exc_info())
            continue

        experiment_finished = False
        for msg in run_msgs['/experiment']:
            if msg.event == mrta.msg.ExperimentEvent.END_EXPERIMENT:
                experiment_finished = True

        if not experiment_finished:
            print("Experiment timed out! Skipping...")
            continue

        bag_filename = os.path.basename(bag_path)
        (map, start_config, mechanism, scenario_id, remainder) = bag_filename.split('__')

        scenario_id = scenario_id.replace('.yaml', '')

        if scenario_id not in target_point_configs:
            read_point_config(task_db, scenario_id)

        # Create a Cairo surface and get a handle to its context
        surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, IMG_WIDTH, IMG_HEIGHT)
        # print("Opening map file {0}".format(map_file))
        # surface = cairo.ImageSurface.create_from_png(map_file)

        ctx = cairo.Context(surface)

        # Invert the y-axis to make life easier
        ctx.transform(cairo.Matrix(yy=-1, y0=IMG_HEIGHT))

        ctx.scale(IMG_WIDTH, IMG_HEIGHT)

        # Paint a white background
        # ctx.set_source_rgb(1.0, 1.0, 1.0)
        # ctx.rectangle(0, 0, 1.0, 1.0)
        # ctx.fill()

        # draw_arena(ctx)

        # Draw target points
        target_points = target_point_configs[scenario_id]

        run_msgs = defaultdict(list)

        try:
            for topic, msg, msg_time in bag.read_messages():
                run_msgs[topic].append(msg)

            draw_edges(ctx, scenario_id)
            draw_target_points(ctx, scenario_id, run_msgs)
        except:
            print("Couldn't read messages from {0}!".format(bag_path))
            traceback.print_exc()
            continue

        bag_basename = bag_filename.replace('.bag', '')
        bag_basename = bag_basename.replace('.yaml', '')

        plot_filename = 'task_graph__' + bag_basename + '.png'
        plot_filename = os.path.join(traj_dir, plot_filename)

        # print("Writing {0}".format(plot_filename))
        # surface.write_to_png(plot_filename)

        # Crop the full image
        output_surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, 1400, 2000)
        out_ctx = cairo.Context(output_surface)

        out_ctx.set_source_surface(surface, 0, -600)
        out_ctx.paint()

        output_surface.write_to_png(plot_filename)

        stop_ros()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot the task graph for the given experiments (bags).')

    parser.add_argument('bag_file',
                        nargs='+',
                        help='Path to the bag file of the experiment to plot.')

    parser.add_argument("--task_dir",
                        nargs='?',
                        default=TASK_FILES_DEFAULT,
                        help="Location of task configuration (target point) files")

    parser.add_argument('--map_file', '-m',
                        help="Path to the map file to draw on top of")

    args = parser.parse_args()

    bag_files = args.bag_file
    task_dir = args.task_dir
    map_file = args.map_file

#    pp.pprint(bag_files)

    # Make one, flat list of paths to log files
    bag_paths = []

    for path_arg in bag_files:
        bag_paths.extend(glob.glob(path_arg))

    print("bag paths: ")
    pp.pprint(bag_paths)

    print("task_dir: {0}".format(task_dir))

    plot_task_graph(bag_paths, task_dir, map_file)
