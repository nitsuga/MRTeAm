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
from collections import defaultdict
import geometry_msgs.msg
import move_base_msgs.msg
import nav_msgs.msg
import rosgraph
import rosgraph.names
import rosnode
import rospy
import unique_id

import multirobot_common
import multirobot_common.msg

## TODO:
# Import Task class

# We'll sleep 1/RATE seconds in every pass of the idle loop.
RATE = 10

pp = pprint.PrettyPrinter(indent=2)

def on_sigint(signal, frame):
    print('Caught SIGINT, shutting down...')
    sys.exit(0)


class Auction(object):
    def __init__(self, auctioneer=None, tasks=None, auction_round=None):
        
        # A handle to the Auctioneer object who called us
        self.auctioneer = auctioneer

        # The tasks we are meant to announce/award in the current round
        self.tasks = tasks

        # To identify in which round bids are made for tasks
        self.auction_round = auction_round

        # Set up state machine.
        # See multirobot/docs/auctioneer-fsm.png
        self.fsm = Fysom( 
            events=[
                ('startup', 'none', 'announce'),
                ('announced', 'announce', 'collect_bids'),
                ('bids_collected', 'collect_bids', 'determine_winner'),
                ('winner_determined', 'determine_winner', 'award'),
            ],
            callbacks={
                # on-enter state handlers
                'onannounce': self.announce,
                'oncollect_bids': self.collect_bids,
                'ondetermine_winner': self.determine_winner,
                'onaward': self.award
                
                # on-event handlers
                #'onchoose_OSI': self.choose_OSI
            }
        )

        # Start the state machine
        self.fsm.startup()

    def _construct_task_msg(self, task):
        """
        Maps from our internal task representation to a ROS message type.
        (multirobot_common.SensorSweepTask => multirobot_common.msg.SensorSweepTask)
        """
        # Just sensor sweep tasks for now
        task_msg = multirobot_common.msg.SensorSweepTask()

        task_msg.task.task_id = task.task_id
        task_msg.task.depends = task.depends
        task_msg.task.type = task.type
        task_msg.task.num_robots_required = task.num_robots_required
        task_msg.location.x = task.location.x
        task_msg.location.y = task.location.y
        task_msg.location.z = task.location.z

        return task_msg

    def _construct_announcement_msg(self):
        pass

    def announce(self, e):
        pass

    def collect_bids(self, e):
        """
        Here, we can either wait for a time limit (deadline) to pass or,
        kowing the size of the team and the mechanism being used, we can
        calculate how many bids we expect to receive before we consider
        the bid collection phase finished.

        Before the time limit passes, we expect to receive bid messages, which
        will trigger Auctioneer.on_bid_received().
        """
        pass
            
    def determine_winner(self, e):
        pass

    def award(self, e):
        pass

class AuctionOSI(Auction):

    mechanism_name = 'OSI'

    def __init__(self, auctioneer=None, tasks=None, auction_round=None):
        super(AuctionOSI, self).__init__(auctioneer, tasks, auction_round)

    def _construct_announcement_msg(self):
        announce_msg = multirobot_common.msg.AnnounceSensorSweep()
        announce_msg.mechanism = self.mechanism_name

        # Only announce one task (the first in the auctioneer's list)
        announce_msg.tasks.append(self._construct_task_msg(self.tasks[0]))

        return announce_msg

    def announce(self, e):
        print("(OSI) state: announce")
        
        announcement_msg = self._construct_announcement_msg()
        self.auctioneer.announce_pub.publish(announcement_msg)

        print('Announcement:')
        pp.pprint(announcement_msg)

        # 
        self.fsm.announced()

    def collect_bids(self, e):
        print("(OSI) state: collect_bids")

        bids = self.auctioneer.bids[self.auction_round]

        # In OSI, we wait to receive as many bids as there are team members
        while len(bids) < len(self.auctioneer.team_members):
            time.sleep(0.2)

        self.fsm.bids_collected(task_id=self.tasks[0].task_id)

    def determine_winner(self, e):
        print("(OSI) state: determine_winner")
        
        # Get the id of the task to award from the event object
        task_id = e.task_id

        bids = self.auctioneer.bids[self.auction_round]

        winner_id = None
        minimum_bid = None
        for robot_id in bids.keys():
            if minimum_bid is None or bids[robot_id][task_id] < minimum_bid:
                winner_id = robot_id

        print("winner is robot '{0}'".format(winner_id))

        self.fsm.winner_determined(task_id=task_id, winner_id=winner_id)

    def award(self, e):
        """ Construct and send award message. """
        award_msg = multirobot_common.msg.TaskAward()
        award_msg.robot_id = e.winner_id

        task_msg = self._construct_task_msg(self.tasks[0])
        award_msg.tasks.append(task_msg)

        self.auctioneer.award_pub.publish(award_msg)

        # Mark the task as awarded
        self.tasks[0].awarded = True

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

        # To identify in which round bids are made for tasks
        self.auction_round = 0

        # Keep track of bids, indexed by auction_round, task_id and robot_id
        self.bids = defaultdict(int)

        # Set up state machine.
        # See multirobot/docs/auctioneer-fsm.png
        self.fsm = Fysom( 
            events=[
                ('startup', 'none', 'load_tasks'),
                ('tasks_loaded', 'load_tasks', 'identify_team'),
                ('do_identify_team', '*', 'identify_team'),
                ('team_identified', 'identify_team', 'idle'),
                ('have_tasks', 'idle', 'choose_mechanism'),
                ('no_tasks', 'idle', 'idle'),
                ('no_tasks', 'choose_mechanism', 'end_experiment'),
            ],
            callbacks={
                # on-enter state handlers
                'onload_tasks': self.load_tasks,
                'onidentify_team': self.identify_team,
                'onidle': self.idle,
                'onchoose_mechanism': self.choose_mechanism,
                'onend_experiment': self.end_experiment,
                # on-event handlers
            }
        )

        # Start the state machine
        self.fsm.startup()

    def init_subscribers(self):
        print 'Initializing subscribers...'

        # Listen for bids on '/tasks/bid'
        self.bid_sub = rospy.Subscriber('/tasks/bid',
                                        multirobot_common.msg.TaskBid,
                                        self.on_bid_received)

    def init_publishers(self):

        # Announce tasks on '/tasks/announce'. For the moment we will only
        # announce sensor sweep tasks.
        self.announce_pub = rospy.Publisher('/tasks/announce',
                                            multirobot_common.msg.AnnounceSensorSweep)

        # Award tasks on '/tasks/award'
        self.award_pub = rospy.Publisher('/tasks/award',
                                         multirobot_common.msg.TaskAward)

        time.sleep(2)

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
                new_task = multirobot_common.SensorSweepTask(str(task_id),
                                                             float(task_x),
                                                             float(task_y))
                self.tasks.append(new_task)

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
                node_name.replace('/rc_', '')
                self.team_members.append(node_name)

        print("Team members: {0}".format(self.team_members))

        self.fsm.team_identified()

    def idle(self, data):
        print "idle.."

        if not self.tasks:
            print "idling..."
            self.rate.sleep()
            self.fsm.no_tasks()

        # Transition to the "choose_mechanism" state
        self.fsm.have_tasks()

    def choose_mechanism(self, e):
        print("Choosing mechanism...")

        # For now, use the single mechanism given to us at startup
        # (in our constructor)
        print("  {0}".format(self.mechanism))


        # As long as there are unallocated tasks, choose a mechanism and
        # allocate them.
        while True:
            unallocated = []
            for task in self.tasks:
                if not task.awarded:
                    unallocated.append(task)

            if not unallocated:
                break

            self.auction_round += 1
            self.bids[self.auction_round] = defaultdict(str)

            if self.mechanism == 'OSI':
                auction_osi = AuctionOSI(self, unallocated, self.auction_round)

                # At this point, we can (safely?) consider a task awarded

            
        self.fsm.no_tasks()

    def on_bid_received(self, bid_msg):
        task_id = bid_msg.task_id
        robot_id = bid_msg.robot_id
        bid = bid_msg.bid

        round_bids = self.bids[self.auction_round]
        
        if not round_bids[robot_id]:
            round_bids[robot_id] = {}

        round_bids[robot_id][task_id] = float(bid)

        print("{0} bid {1} for task {2} in auction round {3}".format(
            robot_id, bid, task_id, self.auction_round))


    def end_experiment(self, e):
        print("end experiment")


if __name__ == '__main__':
    # Exit on ctrl-C
    signal.signal(signal.SIGINT, on_sigint)

    try:
        argv = rospy.myargv(argv=sys.argv[1:])
#        print "arguments: {0}".format(argv)
        auc = Auctioneer(*argv)
    except rospy.ROSInterruptException:
        pass
