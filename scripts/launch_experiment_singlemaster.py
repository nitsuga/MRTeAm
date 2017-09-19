#!/usr/bin/env python

import argparse
import datetime
import mrta
import mrta.msg
import os
import pprint
import rospkg
import rospy
import signal
import subprocess
import sys
import time

# Our random start pose functions
import random_poses

# Paths to ROS binaries
ROS_HOME = '/opt/ros/indigo'
# ROS_HOME = '/home/esch/opt/ros/indigo'
ROSLAUNCH = "{0}/bin/roslaunch".format(ROS_HOME)
# ROSBAG = "{0}/bin/rosbag".format(ROS_HOME)
ROSBAG = "{0}/lib/rosbag/record".format(ROS_HOME)

# How many seconds to wait before killing all processes
TIMEOUT_DEFAULT = 1200
# PSI can take longer than other mechanisms, with the clustered start config
TIMEOUT_PSI = 1200

# Keep track of processes to terminate when done
running_procs = []

# Robots we want to start up
robots = [{'name': 'robot_1',
           'port': '11312'},
          {'name': 'robot_2',
           'port': '11313'},
          {'name': 'robot_3',
           'port': '11314'}]

maps = {
        'brooklyn': 'brooklyn_lab.png',
        'smartlab': {'image': 'smartlab_ugv_arena_v2.png',
                     'yaml': 'smartlab_ugv_arena.yaml',
                     'robot_buffer': 70,
                     'scale': 1.0},
        'strand': {'image': 'map-strand-first-floor-5cm.png',
                   'yaml': 'map-strand-first-floor-5cm.yaml',
                   'robot_buffer': 10,
                   'scale': 6.65},
        'strand-restricted': {'image': 'map-strand-first-floor-restricted-5cm.png',
                              'yaml': 'map-strand-first-floor-restricted-5cm.yaml',
                              'robot_buffer': 10,
                              'scale': 6.65},
        'london': {'image': 'central_london_roads_inverse_2x.png',
                   'yaml': 'central_london.yaml',
                   'robot_buffer': 10,
                   'scale': 1.0},
        'islington': {'image': 'islington_roads_100dpi_inverse.png',
                      'yaml': 'islington.yaml',
                      'robot_buffer': 10,
                      'scale': 3.934}
        }

world_files = {
               'brooklyn': {'clustered': 'brooklyn_arena_3_robots_clustered.world',
                            'distributed': 'brooklyn_arena_3_robots_distributed.world',
                            'distributed_A': ''},
               'smartlab': {'clustered': 'smartlab_ugv_arena_3_robots_clustered.world',
                            'distributed': 'smartlab_ugv_arena_3_robots_distributed.world',
                            'random': 'smartlab_ugv_arena_3_robots_random_starts.world',
                            'start1': 'smartlab_ugv_arena_3_robots_start1.world',
                            'start2': 'smartlab_ugv_arena_3_robots_start2.world',
                            'start3': 'smartlab_ugv_arena_3_robots_start3.world',
                            'start4': 'smartlab_ugv_arena_3_robots_start4.world',
                            'start5': 'smartlab_ugv_arena_3_robots_start5.world',
                            'start6': 'smartlab_ugv_arena_3_robots_start6.world'},

               'strand': {'clustered': 'strand_restricted_3_robots_clustered.world',
                          'random': 'strand_restricted_3_robots_random.world'},
               'strand-restricted': {'clustered': 'strand_restricted_3_robots_clustered.world',
                                     'random': 'strand_restricted_3_robots_random.world'},
               'london': {'clustered': 'central_london.world'},
               'islington': {'clustered': 'islington.world'}
               }

mechanisms = ['OSI', 'PSI', 'SSI', 'RR', 'SUM', 'MAX']

# Topics to record with rosbag
record_topics = ['/experiment', '/tasks/announce', '/tasks/bid',
                 '/tasks/award', '/tasks/status', '/tasks/new', '/debug']

exp_running = False

pp = pprint.PrettyPrinter(indent=2)


def kill_procs():
    for proc in reversed(running_procs):
        try:
            # proc.terminate()
            proc.send_signal(signal.SIGINT)
            proc.wait()
        except:
            print("Error: {0}".format(sys.exc_info()[0]))
        time.sleep(2)


def sig_handler(sig, frame):
    """ Terminate all child processes when we get a SIGINT """
    if sig == signal.SIGINT:
        kill_procs()
        print("Shutting down...")
        sys.exit(0)

signal.signal(signal.SIGINT, sig_handler)          


def on_exp_event(exp_event_msg):
    print("######## on_exp_event() ########")
    print(pp.pformat(exp_event_msg))
    if exp_event_msg.event == mrta.msg.ExperimentEvent.END_EXPERIMENT:
        print("######## END_EXPERIMENT ########")
        global exp_running
        exp_running = False


def launch_experiment(mechanism, map_image, map_yaml, map_scale, robot_buffer, world_file, scenario_id, args):
    rospack = rospkg.RosPack()

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

    reallocate_flag = 'false'
    if args.reallocate:
        reallocate_flag = 'true'

    dynamic_mechanism_flag = 'false'
    try:
        if args.dynamic_mechanism:
            dynamic_mechanism_flag = 'true'
    except AttributeError:
        pass

    if scenario_id == 'none':
        scenario_id = ''

    # If we're doing random start poses, generate them first,
    # but ONLY if we're not "reusing" previously-generated start poses
    if args.start_config == 'random' and not args.reuse_starts:
        print "Generating a new set of random start locations..."
        try:
            os.remove(random_poses.DEFAULT_START_POSE_FILE)
        except OSError:
            pass

        random_poses.generate_and_write_random_starts("{0}/config/maps/{1}".format(rospack.get_path('mrta'), map_image),
                                                      buffer_size=robot_buffer,
                                                      scale=map_scale)

    main_proc = subprocess.Popen([ROSLAUNCH,
                                  main_pkg,
                                  main_launchfile,
                                  "nogui_flag:={0}".format(nogui_flag),
                                  "reallocate:={0}".format(reallocate_flag),
                                  "dynamic_mechanism:={0}".format(dynamic_mechanism_flag),
                                  "map_file:={0}".format(map_yaml),
                                  "world_file:={0}".format(world_file),
                                  "scenario_id:={0}".format(scenario_id),
                                  "mechanism:={0}".format(mechanism),
                                  "classifier_name:={0}".format(args.classifier_name)])
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
    rosbag_args.extend([ '-j',  # compress
                         '-o',  # prepend world, mech and task to filename
                         "{0}__{1}__{2}__{3}_".format(args.map,
                                                      args.start_config,
                                                      mechanism,
                                                      scenario_id)])
    rosbag_args.extend(record_topics)
                                                
    rosbag_proc = subprocess.Popen(rosbag_args)
#    rosbag_proc = subprocess.Popen(' '.join(rosbag_args), shell=True)
    time.sleep(5)

    # Processes are terminated in reverse order of their insertion
    # into running_procs. We add the rosbag process to the list last
    # so that it gets terminated first.
    running_procs.append(rosbag_proc)

    timeout_secs = 0
    if mechanism == 'PSI' and args.start_config == 'clustered':
        timeout_secs = TIMEOUT_PSI
    else:
        timeout_secs = TIMEOUT_DEFAULT

    # Now wait until we receive a SIGINT (Ctrl-C)
    while exp_running:
        time.sleep(1)

        # Time out if the experiment is taking too long
        now = datetime.datetime.now()
        time_delta = now - start_time
        if time_delta.seconds > timeout_secs:
            exp_running = False

    # The experiment is over.
    time.sleep(10)
    print("######## KILLING PROCESSES ########")
    rospy.signal_shutdown('Experiment finished.')
    kill_procs()
    del running_procs[:]
    time.sleep(10)

    # If we did a random start, clean up after ourselves
    if args.start_config == 'random' and not args.reuse_starts:
        os.remove(random_poses.DEFAULT_START_POSE_FILE)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Launch a multirobot task-allocation experiment.')

    parser.add_argument('mechanism',
                        choices=['OSI', 'PSI', 'SSI', 'RR', 'SUM', 'MAX', 'SEL'],
                        help='Mechanism to allocate tasks.')
    parser.add_argument('map',
                        choices=['brooklyn', 'smartlab', 'strand', 'strand-restricted', 'london', 'islington'],
                        help='Map through which the robots move.')
    parser.add_argument('start_config',
                        choices=['clustered', 'distributed', 'random',
                                 'start1', 'start2', 'start3', 'start4', 'start5', 'start6'],
                        help='Starting locations of the robots.')
    parser.add_argument('scenario_id',
                        help='Name of the scenario that defines task locations (and other properties).')
    parser.add_argument("-ng", "--nogui", help="Disable the Stage GUI", action="store_true")
    parser.add_argument("-ra", "--reallocate", help="Re-allocate unfinished tasks", action="store_true")
    parser.add_argument("-rs", "--reuse_starts",
                        help="If using random starting locations, don't generate new locations. Rely on a previously generated start config written to {0}".format(random_poses.DEFAULT_START_POSE_FILE),
                        action="store_true")
    parser.add_argument("-cl", "--classifier_name", type=str,
                        help="Base filename of a classifier to use for dynamic mechanism selection.",
                        default='clf_execution_phase_time_random_forest')

    args = parser.parse_args()

    mechanism = args.mechanism
    map_image = maps[args.map]['image']
    map_yaml = maps[args.map]['yaml']
    robot_buffer = maps[args.map]['robot_buffer']
    map_scale = maps[args.map]['scale']
    world_file = world_files[args.map][args.start_config]
    scenario_id = args.scenario_id

    launch_experiment(mechanism, map_image, map_yaml, map_scale, robot_buffer, world_file, scenario_id, args)
