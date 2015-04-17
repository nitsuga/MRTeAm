#!/usr/bin/env python

import argparse
import datetime
import mrta
import mrta.msg
import pprint
import rospy
import signal
import subprocess
import sys
import time

# Paths to ROS binaries
ROS_HOME = '/opt/ros/hydro'
#ROS_HOME = '/home/esch/opt/ros/hydro'
ROSLAUNCH = "{0}/bin/roslaunch".format(ROS_HOME)
#ROSBAG = "{0}/bin/rosbag".format(ROS_HOME)
ROSBAG = "{0}/lib/rosbag/record".format(ROS_HOME)

# Keep track of processes to terminate when done
running_procs = []

# Robots we want to start up
robots = [ { 'name': 'robot_1',
             'port': '11312' },
           { 'name': 'robot_2',
             'port': '11313' },
           { 'name': 'robot_3',
             'port': '11314' } ]

maps = { 'brooklyn': 'brooklyn_lab.png',
         'smartlab': 'smartlab_ugv_arena_10px_buffers.png' }

world_files = { 'brooklyn': { 'clustered': 'brooklyn_arena_3_robots_clustered.world',
                              'distributed': 'brooklyn_arena_3_robots_distributed.world',
                              'distributed_A': ''},
                'smartlab': { 'clustered': 'smartlab_ugv_arena_3_robots_clustered.world',
                              'distributed': 'smartlab_ugv_arena_3_robots_distributed.world',
                              'distributed_A': 'smartlab_ugv_arena_3_robots_distributed_A.world'} }


mechanisms = [ 'OSI', 'PSI', 'SSI', 'RR' ]

#task_files = [ 'brooklyn_tasks_A.txt',
#               'tasks_A.txt' ]

# Topics to record with rosbag
record_topics = [ '/experiment', '/tasks/announce', '/tasks/bid',
                  '/tasks/award', '/tasks/status' ]

exp_running = False

pp = pprint.PrettyPrinter(indent=2)

def kill_procs():
    for proc in reversed(running_procs):
        try:
            #proc.terminate()
            proc.send_signal(signal.SIGINT)
            proc.wait()
        except:
            print("Error: {0}".format(sys.exc_info()[0]))
        time.sleep(2)

# Terminate all child processes when we get a SIGINT
def sig_handler(sig, frame):
    if sig == signal.SIGINT:
        kill_procs()
        print("Shutting down...")
        sys.exit(0)

signal.signal(signal.SIGINT, sig_handler)          

def on_exp_event(exp_event_msg):
    print("######## on_exp_event() ########")
    print(pp.pformat(exp_event_msg))
    if exp_event_msg.event == 'END_EXPERIMENT':
    #if exp_event_msg.event == 'END_ALLOCATION':
        print("######## END_EXPERIMENT ########")
        global exp_running
        exp_running = False

def launch_experiment(mechanism, map_file, world_file, task_file, args):
    global exp_running
    exp_running = True

    start_time = datetime.datetime.now()

    # Launch the experiment
    print('######## Launching mrta single_master.launch ########')    

    main_pkg = 'mrta'
    main_launchfile = 'single_master.launch'

    nogui_flag = None
    if args.nogui:
      nogui_flag = '-g'

    main_proc = subprocess.Popen([ROSLAUNCH,
                                  main_pkg,
                                  main_launchfile,
                                  "nogui_flag:={0}".format(nogui_flag),
                                  "map_file:={0}".format(map_file),
                                  "world_file:={0}".format(world_file),
                                  "task_file:={0}".format(task_file),
                                  "mechanism:={0}".format(mechanism)])
    running_procs.append(main_proc)
    time.sleep(1)

    # Launch our own node to observe the experiment
    rospy.init_node('experiment_launcher', disable_signals=True)
    rospy.Subscriber('/experiment',
                     mrta.msg.ExperimentEvent,
                     on_exp_event)

    # Start rosbag
    # Record the positions (amcl_pose) of all robots
    for robot in robots:
        record_topics.append("/{0}/amcl_pose".format(robot['name']))

    print('######## Launching rosbag record ########')
    rosbag_args = [ROSBAG, 'record']
    rosbag_args.extend([ '-j',   # compress
                         '-o',   # prepend world, mech and task to filename
                         "{0}__{1}__{2}__{3}_".format(args.map,
                                                  args.start_config,
                                                  mechanism,
                                                  task_file)])
    rosbag_args.extend(record_topics)
                                                
    rosbag_proc = subprocess.Popen(rosbag_args)
#    rosbag_proc = subprocess.Popen(' '.join(rosbag_args), shell=True)
    time.sleep(5)

    # Processes are terminated in reverse order of their insertion
    # into running_procs. We add the rosbag process to the list last
    # so that it gets terminated first.
    running_procs.append(rosbag_proc)

    # Now wait until we receive a SIGINT (Ctrl-C)
    while exp_running:
        time.sleep(1)

        # Time out if the experiment is taking too long
        now = datetime.datetime.now()
        time_delta = now - start_time
        if time_delta.seconds > 300: # five minutes
            exp_running = False

    # The experiment is over.
    time.sleep(10)
    print("######## KILLING PROCESSES ########")
    rospy.signal_shutdown('Experiment finished.')
    kill_procs()
    del running_procs[:]
    time.sleep(10)

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Launch a multirobot task-allocation experiment.')

    parser.add_argument('mechanism',
                        choices=['OSI','PSI','SSI','RR'],
                        help='Mechanism to allocate tasks.')
    parser.add_argument('map',
                        choices=['brooklyn','smartlab'],
                        help='Map through which the robots move.')
    parser.add_argument('start_config',
                        choices=['clustered','distributed','distributed_A'],
                        help='Starting locations of the robots.')
    parser.add_argument('task_file',
                        help='Name of the file containing task point locations.')
    parser.add_argument("-ng","--nogui", help="Disable the Stage GUI", action="store_true")


    args = parser.parse_args()

    mechanism = args.mechanism
    map_file = maps[args.map]
    world_file = world_files[args.map][args.start_config]
    task_file = args.task_file

    launch_experiment(mechanism, map_file, world_file, task_file, args)
