#!/usr/bin/env python

import argparse
import cairo
from collections import defaultdict
import glob
import math
import mrta
import mrta.msg
import os
from os import listdir
import os.path
from os.path import isdir, isfile, join
import re
import rosbag
import rospkg
import sys
import traceback
import yaml

# For debugging
import pprint
pp = pprint.PrettyPrinter(indent=4)

IMG_WIDTH, IMG_HEIGHT = 800, 600

# Default location of task (target point) files
TASK_FILES_DEFAULT = "{0}/task_files".format(rospkg.RosPack().get_path('mrta_auctioneer'))

stroke_colors = {'robot_1': (1.0, 0.0, 0.0),  # Red
                 'robot_2': (0.0, 1.0, 0.0),  # Green
                 'robot_3': (0.0, 0.0, 1.0)}  # Blue

# Robot start locations
start_locations = {'clustered': ((155.0, 150.0),
                                 (50.0, 150.0),
                                 (50.0, 50.0)),
                   'distributed': ((750.0, 550.0),
                                   (50.0, 550.0),
                                   (50.0, 50.0)),
                   'random': []}

target_point_configs = {}

robot_names = ['robot_1', 'robot_2', 'robot_3']


def _read_points(task_file):
    points = []

    # YAML config file
    if task_file.name.endswith('yaml'):
        yaml_tasks = yaml.load(task_file)

        for yaml_task in yaml_tasks:
            task_loc = yaml_task['location']
            x, y = float(task_loc['x']) * 100., float(task_loc['y']) * 100.
            points.append( (yaml_task['task_id'], x, y) )

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


def read_point_configs(task_dir):

    task_filenames = [ f for f in listdir(task_dir) if isfile(join(task_dir,f)) ]

    for task_filename in task_filenames:
        task_file_path = join(task_dir, task_filename)

        print "Reading points from {0}...".format(task_file_path)
        task_file = open(task_file_path, "rb")
        target_point_configs[task_filename] = _read_points(task_file)


def draw_arena(ctx):
    """ Given a Cairo graphics context (ctx), draw the walls of the arena """
    # Arena line properties
    ctx.set_line_width((3. / IMG_WIDTH))
    ctx.set_source_rgb(0, 0, 0)

    # Draw the outside of the arena
    ctx.rectangle(0, 0, 1, 1)
    ctx.stroke()

    ctx.set_source_rgb(0, 0, 0)

    # Set line end caps to square while drawing the walls
    default_line_cap = ctx.get_line_cap()
    ctx.set_line_cap(cairo.LINE_CAP_SQUARE)

    # Draw the walls
    ctx.move_to(200. / IMG_WIDTH, 500. / IMG_HEIGHT)  # w5
    ctx.line_to(200. / IMG_WIDTH, 400. / IMG_HEIGHT)
    ctx.stroke()

    ctx.move_to(300. / IMG_WIDTH, 500. / IMG_HEIGHT)  # w6
    ctx.line_to(300. / IMG_WIDTH, 400. / IMG_HEIGHT)
    ctx.stroke()

    ctx.move_to(500. / IMG_WIDTH, 600. / IMG_HEIGHT)  # w7
    ctx.line_to(500. / IMG_WIDTH, 400. / IMG_HEIGHT)
    ctx.stroke()

    ctx.move_to(600. / IMG_WIDTH, 500. / IMG_HEIGHT)  # w7
    ctx.line_to(600. / IMG_WIDTH, 400. / IMG_HEIGHT)
    ctx.stroke()

    ctx.move_to(0, 400. / IMG_HEIGHT)  # w8
    ctx.line_to(200. / IMG_WIDTH, 400. / IMG_HEIGHT)
    ctx.stroke()

    ctx.move_to(300. / IMG_WIDTH, 400. / IMG_HEIGHT)  # w9
    ctx.line_to(500. / IMG_WIDTH, 400. / IMG_HEIGHT)
    ctx.stroke()

    ctx.move_to(600. / IMG_WIDTH, 400. / IMG_HEIGHT)  # w10
    ctx.line_to(800. / IMG_WIDTH, 400. / IMG_HEIGHT)
    ctx.stroke()

    ctx.move_to(200. / IMG_WIDTH, 300. / IMG_HEIGHT)  # w11
    ctx.line_to(200. / IMG_WIDTH, 100. / IMG_HEIGHT)
    ctx.stroke()

    ctx.move_to(300. / IMG_WIDTH, 200. / IMG_HEIGHT)  # w12
    ctx.line_to(300. / IMG_WIDTH, 100. / IMG_HEIGHT)
    ctx.stroke()

    ctx.move_to(0, 200. / IMG_HEIGHT)  # w13
    ctx.line_to(200. / IMG_WIDTH, 200. / IMG_HEIGHT)
    ctx.stroke()

    ctx.move_to(300. / IMG_WIDTH, 200. / IMG_HEIGHT)  # w14
    ctx.line_to(500. / IMG_WIDTH, 200. / IMG_HEIGHT)
    ctx.stroke()

    ctx.move_to(600. / IMG_WIDTH, 300. / IMG_HEIGHT)  # w15
    ctx.line_to(600. / IMG_WIDTH, 100. / IMG_HEIGHT)
    ctx.stroke()

    ctx.move_to(500. / IMG_WIDTH, 200. / IMG_HEIGHT)  # w16
    ctx.line_to(500. / IMG_WIDTH, 0)
    ctx.stroke()

    ctx.move_to(600. / IMG_WIDTH, 200. / IMG_HEIGHT)  # w11
    ctx.line_to(800. / IMG_WIDTH, 200. / IMG_HEIGHT)
    ctx.stroke()

    # Restore the default line cap style
    ctx.set_line_cap(default_line_cap)


def draw_start_locs(ctx, start_config, run_msgs):

    # If start poses were random, we need to read through pose messages
    # to find them
    if start_config == 'random':
        # Clear the list of start locations
        del start_locations['random'][:]
        for r_name in robot_names:
            # Get the first pose for each robot
            amcl_msg = run_msgs['/{0}/amcl_pose'.format(r_name)][0]
            amcl_pose = amcl_msg.pose.pose

            # Append a 2-element set to the start_locations['random'] dict entry
            start_locations['random'].append((float(amcl_pose.position.x * 100.0),
                                              float(amcl_pose.position.y * 100.0)))

    # Draw start locations
    for i, start_loc in enumerate(start_locations[start_config]):
        start_loc_x = start_loc[0]
        start_loc_y = start_loc[1]

        # E.g., "robot-1"
        stroke_color = stroke_colors["robot_%d" % (i + 1)]

        ctx.set_source_rgb(stroke_color[0], stroke_color[1], stroke_color[2])
        ctx.set_line_width((3. / IMG_WIDTH))
        ctx.rectangle((start_loc_x - 10) / IMG_WIDTH,
                      (start_loc_y - 10) / IMG_HEIGHT,
                      20. / IMG_WIDTH, 20. / IMG_HEIGHT)
        ctx.stroke()

    # Reset pen to black
    ctx.set_source_rgb(0, 0, 0)


def draw_target_points(ctx, target_points, run_msgs):
    # Font style for printing points
    ctx.select_font_face('Sans', cairo.FONT_SLANT_NORMAL,
                         cairo.FONT_WEIGHT_BOLD)
    ctx.set_font_size(14. / IMG_WIDTH)

    ctx.set_line_width((1. / IMG_WIDTH))

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

    for i, target_point in enumerate(target_points):
        task_id = target_point[0]
        task_x = target_point[1]
        task_y = target_point[2]

        print "plotting task {0}".format(task_id)

        #            print "Drawing task point at (%f,%f)" % (task_x, task_y)

        # Draw point number (i) as a label slightly above and to the right of the point
        ctx.move_to((task_x + 6) / IMG_WIDTH, (task_y + 6) / IMG_HEIGHT)
        ctx.transform(cairo.Matrix(yy=-1, y0=IMG_HEIGHT))
        ctx.show_text(str(i + 1))
        ctx.stroke()
        ctx.transform(cairo.Matrix(yy=-1, y0=IMG_HEIGHT))

        # Draw p-medians as circles
        # if i == 2 or i == 11 or i == 13:
        if task_id in median_task_ids:
            print "task {0} is a median".format(task_id)
            ctx.set_source_rgb(.36, .36, 1.0)
            ctx.set_line_width((3. / IMG_WIDTH))
            ctx.arc(task_x / IMG_WIDTH, task_y / IMG_HEIGHT, 5./IMG_WIDTH, 0, 2*math.pi)
            ctx.stroke()
            ctx.fill()
        else:
            ctx.set_source_rgb(0, 0, 0)
            ctx.set_line_width((1. / IMG_WIDTH))

        ctx.move_to((task_x - 5) / IMG_WIDTH, (task_y - 5) / IMG_HEIGHT)
        ctx.line_to((task_x + 5) / IMG_WIDTH, (task_y + 5) / IMG_HEIGHT)
        ctx.stroke()

        ctx.move_to((task_x - 5) / IMG_WIDTH, (task_y + 5) / IMG_HEIGHT)
        ctx.line_to((task_x + 5) / IMG_WIDTH, (task_y - 5) / IMG_HEIGHT)
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


def draw_paths_to_medians(ctx, start_config, run_msgs, target_points):
    median_distance_re = re.compile('.*\[(\w+)\].*\[(\w+)\] == \[(\d+\.\d+([eE][+-]?\d+)?)\]')

    # Key is robot_id, value is [median task_id, distance]
    robot_medians = defaultdict(str)

    for msg in run_msgs['/debug']:
        if msg.key == 'auctioneer-median-distance':
            matches = re.search(median_distance_re, msg.value)
            if matches:
                msg_robot_id = matches.group(1)
                msg_median_task_id = matches.group(2)
                msg_median_distance = float(matches.group(3))

                if not robot_medians[msg_robot_id]:
                    print "median distance message: {0}".format(msg.value)
                    robot_medians[msg_robot_id] = [msg_median_task_id, msg_median_distance]

    # Make it a dashed grey line
    ctx.set_source_rgb(0.6, 0.6, 0.6)
    ctx.set_dash([5.0 / IMG_WIDTH])

    for robot_num, robot_id in enumerate(robot_names):
        # Hack, sorry
        # start_loc = start_locations[start_config][robot_id]
        start_loc = start_locations[start_config][robot_num]
        median_task_id = robot_medians[robot_id][0]
        distance_to_median = robot_medians[robot_id][1]

        end_loc = None

        for target_point in target_points:
            task_id = target_point[0]
            task_x = target_point[1]
            task_y = target_point[2]

            if task_id == median_task_id:
                end_loc = (task_x, task_y)

        # Draw a line from the robot start location to its median
        ctx.move_to(start_loc[0] / IMG_WIDTH, start_loc[1] / IMG_HEIGHT)
        ctx.line_to(end_loc[0] / IMG_WIDTH, end_loc[1] / IMG_HEIGHT)
        ctx.stroke()

    # Revert to solid black lines
    ctx.set_source_rgb(0, 0, 0)
    ctx.set_dash([])


def draw_trajectories(ctx, run_msgs):
    # Trajectory line width
    ctx.set_line_width((5. / IMG_WIDTH))

    # Get the end time of the execution phase
    exec_end_stamp = None
    # for msg in run_msgs['/experiment']:
    #     if msg.event == mrta.msg.ExperimentEvent.END_EXECUTION:
    #         exec_end_stamp = msg.header.stamp

    # Find the timestamps when the first robot started moving and the last robot stopped moving
    team_move_begin_stamp = None
    team_move_end_stamp = None

    for r_name in robot_names:

        robot_move_begin_stamp = None
        robot_move_end_stamp = None

        last_pose_msg = None
        for amcl_msg in run_msgs['/{0}/amcl_pose'.format(r_name)]:
            if not last_pose_msg:
                last_pose_msg = amcl_msg
                continue

            # If the robot moved
            if not _pose_equal(amcl_msg, last_pose_msg):

                if not robot_move_begin_stamp:
                    robot_move_begin_stamp = amcl_msg.header.stamp

                    if not team_move_begin_stamp or robot_move_begin_stamp < team_move_begin_stamp:
                        team_move_begin_stamp = robot_move_begin_stamp

                robot_move_end_stamp = amcl_msg.header.stamp
                if not team_move_end_stamp or robot_move_end_stamp > team_move_end_stamp:
                    team_move_end_stamp = robot_move_end_stamp

    # print('team_move_begin_stamp: {0}'.format(team_move_begin_stamp))
    # print('team_move_end_stamp: {0}'.format(team_move_end_stamp))

    move_total_time = team_move_end_stamp - team_move_begin_stamp
    move_total_secs = move_total_time.secs + (move_total_time.nsecs/1000000000.)
    # print("move_total_secs: {0}".format(move_total_secs))

    for r_name in robot_names:

        # Stroke color
        stroke_color = stroke_colors[r_name]

        start_pose = None
        num_poses = len(run_msgs['/{0}/amcl_pose'.format(r_name)])

        last_pose = None

        for i,amcl_msg in list(enumerate(run_msgs['/{0}/amcl_pose'.format(r_name)])):

            # alpha = i*1. / num_poses

            pose_stamp = amcl_msg.header.stamp
            # Don't draw anything if we haven't started moving yet
            if pose_stamp < team_move_begin_stamp:
                continue

            pose_elapsed = pose_stamp - team_move_begin_stamp
            pose_elapsed_secs = pose_elapsed.secs + (pose_elapsed.nsecs/1000000000.)

#            print('pose_stamp: {0}'.format(pose_stamp))
#            print('pose_elapsed: {0}'.format(pose_elapsed))
#            print('team_move_begin_stamp: {0}'.format(team_move_begin_stamp))
#             print('pose_elapsed_secs: {0}'.format(pose_elapsed_secs))
#             print('move_total_secs: {0}'.format(move_total_secs))

            # alpha = pose_elapsed_secs / move_total_secs
            alpha = 1.0

            # print('alpha: {0}'.format(alpha))
            ctx.set_source_rgba(stroke_color[0], stroke_color[1], stroke_color[2], alpha)

            amcl_pose = amcl_msg.pose.pose
            pose_x = amcl_pose.position.x
            pose_y = amcl_pose.position.y

            if pose_x == 0 and pose_y == 0:
                continue

            if last_pose is None:
                ctx.move_to(pose_x * 100. / IMG_WIDTH, pose_y * 100. / IMG_HEIGHT)
                # print "{0} start_pose: {1}:".format(r_name, start_pose)
            else:
                ctx.move_to(last_pose.position.x * 100. / IMG_WIDTH, last_pose.position.y * 100. / IMG_HEIGHT)

            ctx.line_to(pose_x * 100. / IMG_WIDTH, pose_y * 100. / IMG_HEIGHT)

            ctx.stroke()

            last_pose = amcl_pose


def plot_trajectory(bag_paths, task_dir):

    # Create the './trajectories' directory if it doesn't already exist
    traj_dir = './trajectories'
    if not os.path.exists(traj_dir):
        os.makedirs(traj_dir)

    read_point_configs(task_dir)

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
        (map, start_config, mechanism, task_file, remainder) = bag_filename.split('__')

        # Create a Cairo surface and get a handle to its context
        surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, IMG_WIDTH, IMG_HEIGHT)
        ctx = cairo.Context(surface)

        # Invert the y-axis to make life easier
        ctx.transform(cairo.Matrix(yy=-1, y0=IMG_HEIGHT))

        ctx.scale(IMG_WIDTH, IMG_HEIGHT)

        # Paint a white background
        ctx.set_source_rgb(1.0, 1.0, 1.0)
        ctx.rectangle(0, 0, 1.0, 1.0)
        ctx.fill()

        draw_arena(ctx)

        # Draw target points
        target_points = target_point_configs[task_file]

        run_msgs = defaultdict(list)

        try:
            for topic, msg, msg_time in bag.read_messages():
                run_msgs[topic].append(msg)

            draw_target_points(ctx, target_points, run_msgs)

            draw_start_locs(ctx, start_config, run_msgs)

            draw_paths_to_medians(ctx, start_config, run_msgs, target_points)

            draw_trajectories(ctx, run_msgs)
        except:
            print("Couldn't read messages from {0}!".format(bag_path))
            traceback.print_exc()
            continue

        bag_basename = bag_filename.replace('.bag', '')
        bag_basename = bag_basename.replace('.yaml', '')

        plot_filename = 'trajectory__' + bag_basename + '.png'
        plot_filename = os.path.join(traj_dir, plot_filename)

        print("Writing {0}".format(plot_filename))
        surface.write_to_png(plot_filename)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot robot trajectories for the given experiments (bags).')

    parser.add_argument('bag_file',
                        nargs='+',
                        help='Path to the bag file of the experiment to plot.')

    parser.add_argument("--task_dir",
                        nargs='?',
                        default=TASK_FILES_DEFAULT,
                        help="Location of task configuration (target point) files")

    args = parser.parse_args()

    bag_files = args.bag_file
    task_dir = args.task_dir

#    pp.pprint(bag_files)

    # Make one, flat list of paths to log files
    bag_paths = []

    for path_arg in bag_files:
        bag_paths.extend(glob.glob(path_arg))

    print("bag paths: ")
    pp.pprint(bag_paths)

    print("task_dir: {0}".format(task_dir))

    plot_trajectory(bag_paths, task_dir)