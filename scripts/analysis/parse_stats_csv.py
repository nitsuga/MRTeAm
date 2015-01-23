#!/usr/bin/env python

from collections import defaultdict
import csv
import glob
import math
import os
import re
import rosbag
import sys

CSV_FILENAME = 'stats.csv'

field_names = [
#    'BAG_FILENAME',
    'DATETIME',
    'MAP',
    'START_CONFIG',
    'MECHANISM',
    'TASK_FILE',
#    'RUN_NUM',
#    'ATTEMPT_NUM',
    'TOTAL_RUN_TIME',
    'DELIBERATION_TIME',
    'EXECUTION_TIME',
    'TOTAL_IDLE_TIME',
    'TOTAL_DELAY_TIME',
    'TOTAL_DISTANCE',
    'TOTAL_COLLISIONS',
    'ANNOUNCE_MSGS',
    'ANNOUNCE_MSG_TASKS',
    'BID_MSGS',
    'ALLOC_MSGS_BYTES',
    'ROBOT1_DISTANCE',
    'ROBOT1_TRAVEL_TIME',
    'ROBOT1_IDLE_TIME',
    'ROBOT1_DELAY_TIME',
    'ROBOT2_DISTANCE',
    'ROBOT2_TRAVEL_TIME',
    'ROBOT2_IDLE_TIME',
    'ROBOT2_DELAY_TIME',
    'ROBOT3_DISTANCE',
    'ROBOT3_TRAVEL_TIME',
    'ROBOT3_IDLE_TIME',
    'ROBOT3_DELAY_TIME',
]

ROBOT_NAMES = [ 'robot_1',
                'robot_2',
                'robot_3' ]

class Robot(object):
    def __init__(self):
        self.distance = 0.0
        self.travel_time = 0.0
        self.idle_time = 0.0
        self.delay_time = 0.0


def usage():
    print 'Usage: ' + sys.argv[0] + ': <path to .bag file(s)>'

def main(argv):

    if len(argv) < 1:
        usage()
        sys.exit(1)

    # Make one, flat list of paths to log files
    bag_paths = []

    for path_arg in argv:
        bag_paths.extend(glob.glob(path_arg))

    # Open the CSV file for writing
    csv_file = csv.writer(open(CSV_FILENAME, 'wb'))
    csv_file.writerow(field_names)

    dt_re = re.compile('(.*)\.bag')

    for i,bag_path in enumerate(bag_paths):
        print("Reading {0}".format(bag_path))

        bag = None
        try:
            bag = rosbag.Bag(bag_path)
        except:
            print("Couldn't open {0} for reading!".format(bag_path))
            continue
        
        row_fields = []
        bag_filename = os.path.basename(bag_path)
        #row_fields.append(bag_filename)

        (map, start_config, mechanism, task_file, remainder) = bag_filename.split('__')
        
        dt_match = dt_re.search(remainder)

        # Date/time
        row_fields.append(dt_match.group(1)) # 'DATETIME'

        # 'START_CONFIG', 'MECHANSIM', 'TASK_FILE'
        row_fields.extend([map, start_config, mechanism, task_file])

        run_msgs = defaultdict(list)

        for topic,msg,msg_time in bag.read_messages():
            run_msgs[topic].append(msg)

        exp_msgs = {}
        for msg in run_msgs['/experiment']:
            exp_msgs[msg.event] = msg

        # Total run time
        total_diff = (exp_msgs['END_EXPERIMENT'].header.stamp - 
                      exp_msgs['BEGIN_EXPERIMENT'].header.stamp)
        total_run_time = (total_diff.secs + total_diff.nsecs/1000000000.)
        row_fields.append(total_run_time) # 'TOTAL_RUN_TIME'

        # Deliberation time
        delib_diff = (exp_msgs['END_ALLOCATION'].header.stamp - 
                      exp_msgs['BEGIN_ALLOCATION'].header.stamp)
        delib_time = (delib_diff.secs + delib_diff.nsecs/1000000000.)
        row_fields.append(delib_time) # 'DELIBERATION_TIME'

        # Execution time
        exec_diff = (exp_msgs['END_EXECUTION'].header.stamp - 
                      exp_msgs['BEGIN_EXECUTION'].header.stamp)
        exec_time = (exec_diff.secs + exec_diff.nsecs/1000000000.)
        row_fields.append(exec_time) # 'EXECUTION_TIME'        

        total_idle_time = 0.0
        total_delay_time = 0.0
        total_distance = 0.0
        total_collisions = 0

        robots = {}
        for r_name in ROBOT_NAMES:
            robot = Robot()

            # Distance travelled
            robot.distance = 0.0
            last_pose = None
            for amcl_msg in run_msgs['/{0}/amcl_pose'.format(r_name)]:

                amcl_pose = amcl_msg.pose.pose
                if amcl_pose.position.x == 0 and amcl_pose.position.y == 0:
                    continue

                if last_pose is None:
                    last_pose = amcl_pose
                    continue

                pos_delta = math.hypot(amcl_pose.position.x - last_pose.position.x,
                                       amcl_pose.position.y - last_pose.position.y)
                robot.distance += pos_delta

            #print "{0} distance: {1}".format(r_name, distance)
            total_distance += robot.distance
            
            # Travel time
            robot.travel_begin_time = None
            robot.travel_end_time = None
            for status_msg in run_msgs['/tasks/status']:

                if status_msg.robot_id != r_name:
                    continue

                if status_msg.status == 'BEGIN' and robot.travel_begin_time is None:
                    robot.travel_begin_time = status_msg.header.stamp

                if status_msg.status == 'ALL_TASKS_COMPLETE':
                    robot.travel_end_time = status_msg.header.stamp

            travel_time_diff = robot.travel_end_time - robot.travel_begin_time
            robot.travel_time = (travel_time_diff.secs + travel_time_diff.nsecs/1000000000.)

            idle_time_diff = exp_msgs['END_EXECUTION'].header.stamp - robot.travel_end_time
            robot.idle_time = (idle_time_diff.secs + idle_time_diff.nsecs/1000000000.)

            total_idle_time += robot.idle_time

            # Need to revise this!
            robot.collisions = 0
            total_collisions += robot.collisions

            # Need to revise this!
            robot.delay_time = 0.0
            total_delay_time += robot.delay_time

            robots[r_name] = robot

        row_fields.append(total_idle_time)  # 'TOTAL_IDLE_TIME'
        row_fields.append(total_delay_time) # 'TOTAL_DELAY_TIME'
        row_fields.append(total_distance)   # 'TOTAL_DISTANCE'
        row_fields.append(total_collisions) # 'TOTAL_COLLISIONS'

        # Number of announcement messages
        num_announcements = len(run_msgs['/tasks/announce'])
        row_fields.append(num_announcements) # 'ANNOUNCEMENTS'

        # Number of tasks announced in all announcements
        num_ann_tasks = 0
        alloc_msgs_bytes = 0
        for ann_msg in run_msgs['/tasks/announce']:
            num_ann_tasks += len(ann_msg.tasks)
            alloc_msgs_bytes += sys.getsizeof(ann_msg)

        row_fields.append(num_ann_tasks) # 'ANNOUNCE_MSG_TASKS'

        num_bid_msgs = 0
        for bid_msg in run_msgs['/tasks/bid']:
            num_bid_msgs += 1
            alloc_msgs_bytes += sys.getsizeof(bid_msg)

        row_fields.append(num_bid_msgs) # 'BID_MSGS'

        for award_msg in run_msgs['/tasks/award']:
            alloc_msgs_bytes += sys.getsizeof(award_msg)

        row_fields.append(alloc_msgs_bytes) # 'ALLOC_MSGS_BYTES'

        for r_name in ROBOT_NAMES:
            robot = robots[r_name]

            row_fields.append(robot.distance)     # 'ROBOT<n>_DISTANCE'
            row_fields.append(robot.travel_time)  # 'ROBOT<n>_TRAVEL_TIME'
            row_fields.append(robot.delay_time)   # 'ROBOT<n>_IDLE_TIME'
            row_fields.append(robot.distance)     # 'ROBOT<n>_DELAY_TIME'


        csv_file.writerow(row_fields)

if __name__ == '__main__':
    main(sys.argv[1:])
