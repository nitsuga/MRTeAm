#!/usr/bin/env python

import signal
import subprocess
import sys
import time

# Paths to ROS binaries
ROS_HOME = '/opt/ros/hydro'
ROSLAUNCH = "{0}/bin/roslaunch".format(ROS_HOME)
ROSBAG = "{0}/bin/rosbag".format(ROS_HOME)

# Keep track of processes to terminate when done
running_procs = []

# Robots we want to start up
robots = [ { 'name': 'robot_0',
             'port': '11312' },
           { 'name': 'robot_1',
             'port': '11313' },
           { 'name': 'robot_2',
             'port': '11314' } ]

arenas = { 'brooklyn': { 'map_file': 'brooklyn_lab.png',
                         'world_file': 'brooklyn_arena_3_robots_distributed.world' },
           'smartlab': { 'map_file': 'smartlab_ugv_arena_10px_buffers.png',
                         'world_file': 'smartlab_ugv_arena_3_robots_distributed.world' } }

mechanisms = [ 'OSI', 'PSI', 'SSI', 'RR' ]

task_files = [ 'brooklyn_tasks_A.txt',
               'tasks_A.txt' ]

# Topics to record with rosbag
record_topics = [ '/experiment', '/tasks/announce', '/tasks/bid',
                  '/tasks/award', '/tasks/status' ]

# Terminate all child processes when we get a SIGINT
def sig_handler(signal, frame):
    if signal == signal.SIGINT:
        print("Shutting down...")
        
        for proc in running_procs:
            try:
                proc.terminate()
            except:
                print("Error: {0}".format(sys.exc_info()[0]))

signal.signal(signal.SIGINT, sig_handler)          

def launch_experiments(arena, mechanism, task_file):

    # Launch the "main" roscore on port 11311 : stage_ros and map_server
    main_pkg = 'multirobot_stage'
    main_launchfile = 'stage-3-robots.launch'
    map_file = arena['map_file']
    world_file = arena['world_file']

    main_proc = subprocess.Popen([ROSLAUNCH,
                                  main_pkg,
                                  main_launchfile,
                                  "map_file:={0}".format(map_file),
                                  "world_file:={0}".format(world_file)])
    running_procs.append(main_proc)
    time.sleep(6)

    # Start rosbag
    # Record the positions (odometry) of all robots
    for robot in robots:
        record_topics.append("/{0}/odom".format(robot['name']))

    rosbag_proc = subprocess.Popen([ROSBAG, 'record'] + record_topics + 
                                   [ '-j',   # compress
                                     '-o',   # prepend world, mech and task to filename
                                     "{0}-{1}-{2}".format(world_file,
                                                          mechanism,
                                                          task_file)])
    running_procs.append(rosbag_proc)
    time.sleep(5)

    # Launch the robots
    robot_pkg = 'multirobot_stage'
    robot_launchfile = 'robot_move_base.launch'
    for robot in robots:
        robot_proc = subprocess.Popen([ROSLAUNCH,
                                       robot_pkg,
                                       robot_launchfile,
                                       '-p', robot['port'],
                                       "robot_name:={0}".format(robot['name'])])
        running_procs.append(robot_proc)
        time.sleep(10)

    # Launch the auctioneer
    auc_pkg = 'auctioneer'
    auc_launchfile = 'auctioneer.launch'
    auc_port = '11315'
    mechanism = mechanism
    task_file = task_file
    auc_proc = subprocess.Popen([ROSLAUNCH,
                                 auc_pkg,
                                 auc_launchfile,
                                 '-p', auc_port,
                                 "mechanism:={0}".format(mechanism),
                                 "task_file:={0}".format(task_file)])

    running_procs.append(auc_proc)
    
    # Now wait until we receive a SIGINT (Ctrl-C)
    while True:
        time.sleep(1)


if __name__ == '__main__':
                             
    arena = arenas['brooklyn']
    mechanism = mechanisms[0]
    task_file = task_files[0]    # brooklyn_tasks_A.txt

    launch_experiments(arena, mechanism, task_file)
