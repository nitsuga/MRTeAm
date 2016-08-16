import argparse
from PIL import Image
import random
import sys

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

    # The random poses (pairs of (x,y)) to return, initially empty
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
                # Add (candidate_x,candidate_y) to the list of poses to return
                # Remember that the y-axis starts from the top, so 'reverse' the value here
                random_poses.append((candidate_x, (im_height - candidate_y)))

                # 'Occupy' a region around the point with a certain color,
                # in our case RGBA black (0, 0, 0, 255)
                occupied_color = (0, 0, 0, 255)
                occupy_region(image, candidate_x, candidate_y, robot_size, occupied_color)

                break

    return random_poses


def main(map_image_file, num_robots, robot_size, output_file):

    try:
        map_image = Image.open(map_image_file)
    except:
        print "Couldn't open {0} for reading!".format(map_image_file)
        sys.exit(1)

     random_poses = get_random_poses(map_image, num_robots, robot_size)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Generate a stage world file containing random starting poses.')

    # Fix this
    #parser.add_argument('num_robots',
    #                    choices=['OSI','PSI','SSI','RR'],
    #                    help='Mechanism to allocate tasks.')

    args = parser.parse_args()

    map_image_file = args.map_image_file
    num_robots = args.num_robots
    robot_size = args.robot_size
    output_file = args.output_file

    main(map_image_file, num_robots, robot_size, output_file)
