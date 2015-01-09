#!/usr/bin/env python

"""auctioneer.py

This program serves as the task-allocating agent in a multirobot system. It is
responsible for:

  1. Loading a group of tasks that make up a "mission". These tasks may have
     dependencies among them (e.g., ordering), real-time constraints (e.g.,
     deadlines or windows), or other constraints. By "loading", we mean that
     tasks are read from a task- or mission-definition file at startup, received
     online from a mission-controlling agent (e.g., a person operating a GUI).

  2. Announcing tasks (in a particular order, if constrained) to team members.
     Tasks may be announced in groups, or "bundles".

  3. Receiving bids on tasks from team members. A bid is a real numbered value
     that (for now) represents the cost that a team member estimates it will
     incur to complete the task. For example, the distance it will need to
     travel to reach a certain point in the world.

  4. Determining the "winner" of every task. A winner is usually the team
     member that submitted the lowest bid.

The term "mechanism" refers to the way in which tasks are announced, bids collected,
and tasks are awarded. Depending on the mechanism, the announcement and bid collection
phases may be skipped (e.g., in a round-robin mechanism).

Usage: auctioneer.py ['RR'|'PSI'|'OSI'|'SSI'] [path to task definition file]

  The first argument specifies the mechanism to use. These abbreviations stand for
  "round robin", "parallel single-item", "ordered single-item", and "sequential
  single-item", respectively. These mechanisms are explained in [1].

  The second argument gives the path to a file whose contents specify the
  set of tasks that make up a mission. The format of this file is explained
  in [TODO].


Eric Schneider <eric.schneider@liverpool.ac.uk>

[1] Schneider, Balas, et al. 2014. An empirical evaluation of auction-based task allocation in multi-robot teams. In Proceedings of the 2014 international conference on Autonomous agents and multi-agent systems (AAMAS '14).
"""

# Standard Python modules
import os
import pprint
import signal
import sys
import time
import uuid

# Fysom state machine
from fysom import Fysom

# ROS modules
import actionlib
import geometry_msgs.msg
import move_base_msgs.msg
import nav_msgs.msg
import rosgraph
import rosgraph.names
import rosnode
import rospy
import unique_id

import multirobot_common.msg

## TODO:
# Import Task class

# We'll sleep 1/RATE seconds in every pass of the idle loop.
RATE = 10

pp = pprint.PrettyPrinter(indent=2)

def on_sigint(signal, frame):
    print('Caught SIGINT, shutting down...')
    sys.exit(0)

class Auctioneer:

    def __init__(self, mechanism=None, task_file=None):
        """
        Initialize some ROS stuff (topics to publish/subscribe) our state machine.
        """

        # Initialize our node
        # Do we need a special name for the auctioneer (i.e., not "auctioneer")?
        node_name = 'auctioneer'
        #rospy.loginfo("Starting node '{0}'...".format(node_name))
        print("Starting node '{0}'...".format(node_name))
        rospy.init_node(node_name)

        # Topics we wish to subscribe to
        self.init_subscribers()

        # Topics we wish to publish
        self.init_publishers()

        # The rate at which we'll sleep while idle
        self.rate = rospy.Rate(RATE)

        # A list of (node) names of robot team members.
        self.team_members = []

        # We will use one mechanism per run (for now)
        self.mechanism = mechanism

        # Tasks are loaded from a configuration file, rather than
        # loaded dynamically (for now)
        self.task_file = task_file

        # A simple list for now
        self.tasks = []

        # See multirobot/docs/auctioneer-fsm.png
#        self.fsm = Fysom( initial='load_tasks',
        self.fsm = Fysom( 
                          events=[
                              ('startup', 'none', 'load_tasks'),
                              ('tasks_loaded', 'load_tasks', 'identify_team'),
                              ('team_identified', 'identify_team', 'idle'),
                              ('no_tasks', '*', 'idle'),
                              ('have_tasks', 'idle', 'choose_mechanism'),
                              
                              ('choose_OSI', 'choose_mechanism', 'announce'),
                              ('choose_PSI', 'choose_mechanism', 'announce'),
                              ('choose_SSI', 'choose_mechanism', 'announce'),
                              ('choose_RR', 'choose_mechanism', 'determine_winner'),
                              
                              ('announced', 'announce', 'collect_bids'),
                              ('bids_collected', 'collect_bids', 'determine_winner'),
                              ('winner_determined', 'determine_winner', 'award'),
                              ('have_tasks', 'award', 'choose_mechanism')
                          ],
                          callbacks={
                              # on-enter state handlers
                              'onload_tasks': self.load_tasks,
                              'onidentify_team': self.identify_team,
                              'onidle': self.idle,
                              'onchoose_mechanism': self.choose_mechanism,
                              'onannounce': self.announce,
                              'oncollect_bids': self.collect_bids,
                              'ondetermine_winner': self.determine_winner,
                              'onaward': self.award

                              # on-event handlers
#                              'onchoose_OSI': self.choose_OSI

                          }
                      )

        # Start the state machine
        self.fsm.startup()

    def init_subscribers(self):
        print 'Initializing subscribers...'

        # Listen for bids on '/tasks/bid'
        self.bid_sub = rospy.Subscriber('/tasks/bid',
                                        multirobot_common.msg.TaskBid,
                                        self.on_bid)

    def init_publishers(self):

        # Announce tasks on '/tasks/announce'. For the moment we will only
        # announce sensor sweep tasks.
        self.announce_pub = rospy.Publisher('/tasks/announce',
                                            multirobot_common.msg.AnnounceSensorSweep)

        # Award tasks on '/tasks/award'
        self.award_pub = rospy.Publisher('/tasks/award',
                                         multirobot_common.msg.TaskAward)

    def load_tasks(self, data):
        print("Loading tasks from {0}...".format(self.task_file))

        try:
            task_file = open(self.task_file, 'rb')
        
            # The task file format is simple (for now): each line
            # is a whitespace-sepatated pair of x- and y-coordinates
            # that, together, give the location of a task point

            # 'task_id' is an incrementing integer
            task_id = 1

            for task_line in task_file:
                # Ignore comment lines
                if task_line.startswith('#'):
                    continue

                task_x, task_y = task_line.split()
                
                self.tasks.append({ 'task_id': str(task_id),
                                    'depends': [],
                                    'type': 'SENSOR_SWEEP',
                                    'num_robots_required': 1,
                                    'location': { 'x': float(task_x) / 100.0,
                                                  'y': float(task_y) / 100.0,
                                                  'z': 0.0 }
                                })
                task_id += 1

        except IOError:
            print("Can't open task file {0} for reading!".format(self.task_file))

        #print("Tasks: {0}".format(self.tasks))
        print('Tasks:')
        pp = pprint.PrettyPrinter(indent=2)
        pp.pprint(self.tasks)

        self.fsm.tasks_loaded()

    def identify_team(self, data):
        print "Identifying team..."

        # In the 'rosnode' utility/module, _sub_rosnode_listnodes() returns a
        # newline-separated list of the names of all nodes in the graph.
        # See http://wiki.ros.org/rosnode
        node_list = rosnode._sub_rosnode_listnodes().split()

        # We're looking for nodes whose names begin with '/rc_', a prefix
        # (in the global namespace) that stands for 'robot controller'.
        # For example, 'rc_robot_0'.
        for node_name in node_list:
            if node_name.startswith('/rc_'):
                self.team_members.append(node_name)

        print("Team members: {0}".format(self.team_members))

        self.fsm.team_identified()

    def idle(self, data):
        print "idle.."

        while not self.tasks:
            print "idling..."
            self.rate.sleep()

        # Transition to the "choose_mechanism" state
        self.fsm.have_tasks()

    def choose_mechanism(self, e):
        print("Choosing mechanism...")

        # For now, use the single mechanism given to us at startup
        # (in our constructor)
        print("  {0}".format(self.mechanism))

        # if self.mechanism == 'OSI':
        #     self.fsm.choose_OSI()
        # elif self.mechanism == 'PSI':
        #     self.fsm.choose_PSI()
        # elif self.mechanism == 'SSI':
        #     self.fsm.choose_SSI()
        # elif self.mechanism == 'RR':
        #     self.fsm.choose_RR()

        if self.mechanism in ['RR', 'OSI', 'PSI', 'SSI']:
            self.fsm.trigger('choose_' + self.mechanism)

    def _construct_task_msg(self, task):
        """
        Maps from our internal task representation to a ROS message type
        (multirobot_common/Task)
        """

        # Just sensor sweep tasks for now
        task_msg = multirobot_common.msg.SensorSweepTask()

        task_msg.task.task_id = task['task_id']
        task_msg.task.depends = task['depends']
        task_msg.task.type = task['type']
        task_msg.task.num_robots_required = task['num_robots_required']
        task_msg.location.x = task['location']['x']
        task_msg.location.y = task['location']['y']
        task_msg.location.z = task['location']['z']

        return task_msg

    def _construct_announcement_msg(self, mechanism):
        announce_msg = multirobot_common.msg.AnnounceSensorSweep()

        announce_msg.mechanism = self.mechanism

        tasks_to_announce = None

        if mechanism == 'OSI':
            tasks_to_announce = [self.tasks[0]]
        elif mechanism == 'PSI':
            tasks_to_announce = self.tasks
        elif mechanism == 'SSI':
            tasks_to_announce = self.tasks

#        print("tasks_to_announce:")
#        pp.pprint(tasks_to_announce)
            
        for task in tasks_to_announce:
            announce_msg.tasks.append(self._construct_task_msg(task))

        return announce_msg

    # def choose_OSI(self, e):
    #     print("choose_OSI")

    #     # Announce the first task in the list
    #     announcement = self._construct_announcement(self.tasks[0])

    def announce(self, e):
        announcement_msg = self._construct_announcement_msg(self.mechanism)

        print('Announcement:')
        pp.pprint(announcement_msg)

        self.announce_pub.publish(announcement_msg)

        self.fsm.announced()


    def collect_bids(self, e):
        time.sleep(5)

        self.fsm.bids_collected()

    def on_bid(self, data):
        # TODO: implement!
        pass


    def determine_winner(self, e):
        
        self.fsm.winner_determined()

    def award(self, e):
        # send award message

        if self.tasks:
            self.fsm.have_tasks()
        else:
            self.fsm.no_tasks()


if __name__ == '__main__':
    # Exit on ctrl-C
    signal.signal(signal.SIGINT, on_sigint)

    try:
        argv = rospy.myargv(argv=sys.argv[1:])
#        print "arguments: {0}".format(argv)
        auc = Auctioneer(*argv)
#        auc.spin()
    except rospy.ROSInterruptException:
        pass
