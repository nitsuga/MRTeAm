#!/usr/bin/env python

import rosbag
import sys
import textwrap

exp_topics = { 'experiment': [ '/experiment' ],
               'tasks': [ '/tasks/announce',
                          '/tasks/award',
                          '/tasks/bid',
                          '/tasks/status' ],
               'position': [ '/robot_0/odom',
                             '/robot_1/odom',
                             '/robot_2/odom' ] }
                     

# For textwrap
preferred_width = 80

def usage(script):
    print("Usage: {0} <bag_file> <'experiment'|'tasks'|'position'>".format(script))

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
        sys.exit(1)

    for topic,msg,t in bag.read_messages(topics=exp_topics[argv[2]]):
        print("{0}.{1}: ".format(t.secs,t.nsecs))
        print(msg)
        print('------------------------------------------------')

if __name__ == '__main__':
    main(sys.argv)
