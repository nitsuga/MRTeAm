#!/usr/bin/env python

import argparse
from PIL import Image
import random
import sys

DEFAULT_TMP_FILE = '/tmp/robot_random_starts.inc'

def is_valid_color(color, threshold):
    """ Return True if each component of the given color (r, g, b, a) is
        above threshold
    """
    if color[0] > threshold and color[1] > threshold and color[2] > threshold:
        return True

    return False


def is_valid_point(image, candidate_x, candidate_y, buffer):
    """ Return True if the point at (x, y) and a buffer-sized area around it
        are unoccupied

    :param image: the PIL.Image file to search
    :param candidate_x: x-coordinate of a candidate point
    :param candidate_y: y-coordinate of a candidate point
    :param buffer: nominal diameter of a robot
    """

    # We assume pure white (255, 255, 255) means unoccupied.
    # Set our threshold to 254
    threshold = 254

    for x in range((candidate_x - buffer / 2), (candidate_x + buffer / 2)):
        for y in range((candidate_y - buffer / 2), (candidate_y + buffer / 2)):

            # print "({0}, {1})".format(x, y)
            try:
                if not is_valid_color(image.getpixel((x, y)), threshold):
                    return False
            except IndexError:
                # We tried to read a pixel out of bounds (e.g. a negative coordinate)
                continue

    return True


def occupy_region(image, center_x, center_y, buffer, color):
    """ 'Occupy' a region around (x,y) with the given color """

    for x in range((center_x - buffer / 2), (center_x + buffer / 2)):
        for y in range((center_y - buffer / 2), (center_y + buffer / 2)):

            try:
                image.putpixel((x, y), color)
            except IndexError:
                # We tried to modify a pixel out of bounds (e.g. a negative coordinate)
                continue


def get_random_poses(image, num_robots, robot_size):
    im_width, im_height = image.size

    # The random poses (pairs of (x, y, z, alpha)) to return, initially empty
    # z is always 0 and alpha is in the range [-180, 180]
    random_poses = []

    for i in range(num_robots):
        # Have we found a valid point for the robot?
        valid_point = False

        while not valid_point:

            candidate_x = random.randint(0, im_width + 1)
            # Y-coordinates start at the top of the image, not the bottom
            candidate_y = random.randint(0, im_height + 1)

            # We found a valid point
            if is_valid_point(image, candidate_x, candidate_y, robot_size):
                alpha = round(random.uniform(-180, 180), 2)
                # Add (candidate_x, candidate_y, 0, alpha) to the list of poses to return
                # Remember that the y-axis starts from the top, so 'reverse' the value here.
                # Also, Stage needs values in meters, not cm, so divide by 100 here
                reversed_y = im_height - candidate_y
                random_poses.append((candidate_x/100.0, reversed_y/100.0, 0, alpha))

                # 'Occupy' a region around the point with a certain color,
                # in our case RGBA black (0, 0, 0, 255)
                occupied_color = (0, 0, 0, 255)
                occupy_region(image, candidate_x, candidate_y, robot_size, occupied_color)

                break

    return random_poses


def generate_and_write(map_image_file, num_robots=3, robot_size=70, output_filename=DEFAULT_TMP_FILE):

    print "map = {0}, num = {1}, size = {2}, output = {3}".format(map_image_file,
                                                                  num_robots,
                                                                  robot_size,
                                                                  output_filename)

    try:
        map_image = Image.open(map_image_file)
    except IOError:
        print "Couldn't open map image {0} for reading!".format(map_image_file)
        sys.exit(1)

    random_poses = get_random_poses(map_image, num_robots, robot_size)

    # Write the poses to output_file
    try:
        output_file = open(output_filename, 'wb')

        robot_colors = ['red', 'green', 'blue']

        for i, pose in enumerate(random_poses):
            print "pose: [{0}, {1}, {2}, {3}]".format(*pose)
            output_file.write("turtlebot ( pose [ {0} {1} {2} {3} ] color \"{4}\" )\n".format(pose[0],
                                                                                            pose[1],
                                                                                            pose[2],
                                                                                            pose[3],
                                                                                            robot_colors[i]))

    except IOError:
        print "Couldn't open {0} for writing! Exiting.".format(output_filename)
        sys.exit(1)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Generate a stage world file containing random starting poses.')

    # Fix this
    parser.add_argument('map_image_file',
                        help='The map file (e.g., .png) to use.')

    parser.add_argument('-n', '--num_robots',
                        help='The number of robots/poses to generate',
                        default=3)

    parser.add_argument('-s', '--robot_size',
                        help='(Square) robot footprint in cm/pixels',
                        default=70)

    parser.add_argument('-o', '--output_file',
                        help='Name of the Stage .inc file to write robot poses.',
                        default="/tmp/robot_random_starts.inc")

    args = parser.parse_args()

    map_image_file = args.map_image_file
    num_robots = args.num_robots
    robot_size = args.robot_size
    output_file = args.output_file

    generate_and_write(map_image_file, num_robots, robot_size, output_file)
