#!/usr/bin/env python

import rosbag
import sys
import pprint

pp = pprint.PrettyPrinter(indent=4)

exp_topics = { 'experiment': [ '/experiment' ],
               'tasks': [ '/tasks/announce',
                          '/tasks/award',
                          '/tasks/bid',
                          '/tasks/status' ],
               'announcements': ['/tasks/announce'],
               'bids': ['/tasks/bid'],
               'awards': ['/tasks/award'],
               # 'position': [ '/robot_3/amcl_pose'],
               'debug': ['/debug'],
               'position': ['/robot_1/amcl_pose',
                            '/robot_2/amcl_pose',
                            '/robot_3/amcl_pose']}

# For textwrap
preferred_width = 80


def usage(script):
    print("Usage: {0} <bag_file> <'experiment'|'tasks'|'bids'|'awards'|'position'>".format(script))


def main(argv):

    if len(argv) < 3:
        usage(argv[0])
        sys.exit(1)

    bag = None
    try:
        bag = rosbag.Bag(argv[1])
    except IOError:
        print("Can't open {0} as a bag file.".format(argv[1]))
        sys.exit(1)

    if argv[2] not in exp_topics:
        print("Invalid topic type: {0}".format(argv[2]))
        bag.close()
        sys.exit(1)

    for topic, msg, t in bag.read_messages(topics=exp_topics[argv[2]]):
        print('---------------------------------------------------')
        print("time: {0}.{1}s ".format(t.secs, t.nsecs))
        print("topic: {0}".format(topic))
        print(msg)

    bag.close()

if __name__ == '__main__':
    main(sys.argv)
