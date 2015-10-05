#!/usr/bin/env python

import argparse
import cairo
from collections import defaultdict
import glob
import math
import os
from os import listdir
import os.path
from os.path import isdir, isfile, join
import re
import rosbag
import rospkg
import sys
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
                                   (50.0, 50.0))}

target_point_configs = {}

robot_names = ['robot_1', 'robot_2', 'robot_3']


def _read_points(task_file):
    points = []

    # YAML config file
    if task_file.name.endswith('yaml'):
        yaml_tasks = yaml.load(task_file)

        for yaml_task in yaml_tasks:
            task_loc = yaml_task['location']
            points.append( [ float(task_loc['x']), float(task_loc['y']) ] )

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
    ctx.line_to(500. / IMG_WIDTH, 200. / IMG_HEIGHT)
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
    ctx.line_to(200. / IMG_WIDTH, 200. / IMG_HEIGHT)
    ctx.stroke()

    ctx.move_to(300. / IMG_WIDTH, 300. / IMG_HEIGHT)  # w12
    ctx.line_to(300. / IMG_WIDTH, 200. / IMG_HEIGHT)
    ctx.stroke()

    ctx.move_to(0, 200. / IMG_HEIGHT)  # w13
    ctx.line_to(200. / IMG_WIDTH, 200. / IMG_HEIGHT)
    ctx.stroke()

    ctx.move_to(300. / IMG_WIDTH, 200. / IMG_HEIGHT)  # w14
    ctx.line_to(600. / IMG_WIDTH, 200. / IMG_HEIGHT)
    ctx.stroke()

    ctx.move_to(200. / IMG_WIDTH, 100. / IMG_HEIGHT)  # w15
    ctx.line_to(200. / IMG_WIDTH, 0)
    ctx.stroke()

    ctx.move_to(500. / IMG_WIDTH, 100. / IMG_HEIGHT)  # w16
    ctx.line_to(500. / IMG_WIDTH, 0)
    ctx.stroke()

    ctx.move_to(700. / IMG_WIDTH, 200. / IMG_HEIGHT)  # w11
    ctx.line_to(800. / IMG_WIDTH, 200. / IMG_HEIGHT)
    ctx.stroke()

    # Restore the default line cap style
    ctx.set_line_cap(default_line_cap)

def draw_start_locs(ctx, start_config):
    # Draw start locations
    for i, start_loc in enumerate(start_locations[start_config]):
        start_loc_x = start_loc[0]
        start_loc_y = start_loc[1]

        # E.g., "robot-1"
        stroke_color = stroke_colors["robot_%d" % (i + 1)]

        ctx.set_source_rgb(stroke_color[0], stroke_color[1], stroke_color[2])
        ctx.rectangle((start_loc_x - 6) / IMG_WIDTH,
                      (start_loc_y - 6) / IMG_HEIGHT,
                      12. / IMG_WIDTH, 12. / IMG_HEIGHT)
        ctx.stroke()

    # Reset pen to black
    ctx.set_source_rgb(0, 0, 0)

def draw_target_points(ctx, target_points):
    # Font style for printing points
    ctx.select_font_face('Sans', cairo.FONT_SLANT_NORMAL,
                         cairo.FONT_WEIGHT_BOLD)
    ctx.set_font_size(14. / IMG_WIDTH)

    ctx.set_line_width((1. / IMG_WIDTH))

    for i, target_point in enumerate(target_points):
        task_x = target_point[0]
        task_y = target_point[1]

        #            print "Drawing task point at (%f,%f)" % (task_x, task_y)

        # Draw point number (i) as a label slightly above and to the right of the point
        ctx.move_to((task_x + 6) / IMG_WIDTH, (task_y + 6) / IMG_HEIGHT)
        ctx.transform(cairo.Matrix(yy=-1, y0=IMG_HEIGHT))
        ctx.show_text(str(i + 1))
        ctx.stroke()
        ctx.transform(cairo.Matrix(yy=-1, y0=IMG_HEIGHT))

        #            ctx.arc(task_x / IMG_WIDTH, task_y / IMG_HEIGHT, 5./IMG_WIDTH, 0, 2*math.pi)
        #            ctx.stroke()
        #            ctx.fill()
        ctx.move_to((task_x - 5) / IMG_WIDTH, (task_y - 5) / IMG_HEIGHT)
        ctx.line_to((task_x + 5) / IMG_WIDTH, (task_y + 5) / IMG_HEIGHT)
        ctx.stroke()

        ctx.move_to((task_x - 5) / IMG_WIDTH, (task_y + 5) / IMG_HEIGHT)
        ctx.line_to((task_x + 5) / IMG_WIDTH, (task_y - 5) / IMG_HEIGHT)
        ctx.stroke()


def draw_trajectories(ctx, run_msgs):
    # Trajectory line width
    ctx.set_line_width((2. / IMG_WIDTH))

    for r_name in robot_names:

        # Stroke color
        stroke_color = stroke_colors[r_name]
        ctx.set_source_rgb(stroke_color[0], stroke_color[1], stroke_color[2])

        start_pose = None
        for amcl_msg in run_msgs['/{0}/amcl_pose'.format(r_name)]:
            amcl_pose = amcl_msg.pose.pose
            pose_x = amcl_pose.position.x
            pose_y = amcl_pose.position.y

            if pose_x == 0 and pose_y == 0:
                continue

            if start_pose is None:
                ctx.move_to(pose_x * 100. / IMG_WIDTH, pose_y * 100. / IMG_HEIGHT)
                start_pose = amcl_pose
                # print "{0} start_pose: {1}:".format(r_name, start_pose)
            else:
                ctx.line_to(pose_x * 100. / IMG_WIDTH, pose_y * 100. / IMG_HEIGHT)

        ctx.stroke()


def plot_trajectory(bag_paths, task_dir):

    read_point_configs(task_dir)

    for bag_path in bag_paths:
        print("Reading {0}".format(bag_path))

        bag = None
        try:
            bag = rosbag.Bag(bag_path)
        except:
            print("Couldn't open {0} for reading!".format(bag_path))
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

        draw_start_locs(ctx, start_config)

        # Draw target points
        target_points = target_point_configs[task_file]

        draw_target_points(ctx, target_points)

        run_msgs = defaultdict(list)
        for topic, msg, msg_time in bag.read_messages():
            run_msgs[topic].append(msg)

#        draw_trajectories(ctx, run_msgs)

        bag_basename = bag_filename.replace('.bag', '')
        surface.write_to_png(bag_basename + '.png')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Launch a multirobot task-allocation experiment.')

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