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
import itertools
import os
import pprint
import re
from sets import Set
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
import rospy.rostime
import unique_id

import multirobot_common
import multirobot_common.msg

## TODO:
# Import Task class

# We'll sleep 1/RATE seconds in every pass of the idle loop.
RATE = 100

pp = pprint.PrettyPrinter(indent=2)

def on_sigint(signal, frame):
    print('Caught SIGINT, shutting down...')
    sys.exit(0)

def stamp(msg):
    """ Set the timestamp of a message to the current wall-clock time."""
    rospy.rostime.switch_to_wallclock()
    msg.header.stamp = rospy.rostime.get_rostime()

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

    def _get_task_by_id(self, task_id):
        for task in self.tasks:
            if task.task_id == task_id:
                return task

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
        rospy.loginfo("(OSI) state: announce")

        while not self.auctioneer.team_members:
            rospy.logdebug("..waiting for team to be non-empty")
            time.sleep(1)
        
        announcement_msg = self._construct_announcement_msg()
        stamp(announcement_msg)
        self.auctioneer.announce_pub.publish(announcement_msg)

        rospy.logdebug("Announcement:\n{0}".format(pp.pformat(announcement_msg)))

        self.fsm.announced()

    def collect_bids(self, e):
        rospy.loginfo("(OSI) state: collect_bids")

        bids = self.auctioneer.bids[self.auction_round]

        # In OSI, we wait to receive as many bids as there are team members
        while len(bids) < len(self.auctioneer.team_members):
            time.sleep(0.2)

        self.fsm.bids_collected(task_id=self.tasks[0].task_id)

    def determine_winner(self, e):
        rospy.loginfo("(OSI) state: determine_winner")
        
        # Get the id of the task to award from the event object
        task_id = e.task_id

        bids = self.auctioneer.bids[self.auction_round]

        winner_id = None
        minimum_bid = None
        for robot_id in bids.keys():
            bid_value = bids[robot_id][task_id]

            rospy.logdebug("determine_winner(): robot {0} bids {1} on task {2}".format(robot_id,
                                                                                       bid_value,
                                                                                       task_id))
            if minimum_bid is None or bid_value < minimum_bid:
                minimum_bid = bid_value
                winner_id = robot_id

        rospy.loginfo("winner is robot '{0}'".format(winner_id))

        self.fsm.winner_determined(task_id=task_id, winner_id=winner_id)

    def award(self, e):
        """ Construct and send an award message. """
        rospy.loginfo("(OSI) state: award")

        award_msg = multirobot_common.msg.TaskAward()
        award_msg.robot_id = e.winner_id
        task_msg = self._construct_task_msg(self.tasks[0])
        award_msg.tasks.append(task_msg)

        stamp(award_msg)
        self.auctioneer.award_pub.publish(award_msg)

        # Mark the task as awarded
        self.tasks[0].awarded = True

class AuctionPSI(Auction):

    mechanism_name = 'PSI'

    def __init__(self, auctioneer=None, tasks=None, auction_round=None):
        super(AuctionPSI, self).__init__(auctioneer, tasks, auction_round)

    def _construct_announcement_msg(self):
        announce_msg = multirobot_common.msg.AnnounceSensorSweep()
        announce_msg.mechanism = self.mechanism_name

        # Announce all messages at once
        for task in self.tasks:
            announce_msg.tasks.append(self._construct_task_msg(task))

        return announce_msg

    def announce(self, e):
        rospy.loginfo("(PSI) state: announce")

        while not self.auctioneer.team_members:
            rospy.logdebug("..waiting for team to be non-empty")
            time.sleep(1)

        announcement_msg = self._construct_announcement_msg()
        stamp(announcement_msg)
        self.auctioneer.announce_pub.publish(announcement_msg)

        rospy.logdebug("Announcement:\n{0}".format(pp.pformat(announcement_msg)))

        self.fsm.announced()

    def collect_bids(self, e):
        rospy.loginfo("(PSI) state: collect_bids")

        bids = self.auctioneer.bids[self.auction_round]

        # In PSI, the number of bids we expect to receive is [#tasks]*[team size]
        bid_count = 0
        while bid_count < len(self.tasks) * len(self.auctioneer.team_members):
            bid_count = 0
            for robot_id in bids:
                for task_id in bids[robot_id]:
                    bid_count += 1

            time.sleep(0.2)

        self.fsm.bids_collected()

    def determine_winner(self, e):
        rospy.loginfo("(PSI) state: determine_winner")
        
        bids = self.auctioneer.bids[self.auction_round]

        rospy.logdebug("bids:\n{0}".format(pp.pformat(bids)))

        # We'll determine the winner of and send an award message for each task
        task_winners = {} # task_winners[task_id] = winner_id

        for task in self.tasks:

            winner_id = None
            minimum_bid = None
            for robot_id in bids:
                bid_value = bids[robot_id][task.task_id]

                rospy.logdebug("determine_winner(): robot {0} bid {1} on task {2}".format(robot_id,
                                                                                          bid_value,
                                                                                          task.task_id))
                if minimum_bid is None or bid_value < minimum_bid:
                    minimum_bid = bid_value
                    winner_id = robot_id

            rospy.loginfo("winner of task {0} is {1}".format(task.task_id, winner_id))
            task_winners[task.task_id] = winner_id

        self.fsm.winner_determined(task_winners=task_winners)

    def award(self, e):
        """ Construct and send an award message for each winner. """
        rospy.loginfo("(PSI) state: award")

        task_winners = e.task_winners

        for task_id in task_winners:
            won_task = self._get_task_by_id(task_id)
            
            award_msg = multirobot_common.msg.TaskAward()
            award_msg.robot_id = task_winners[task_id]

            task_msg = self._construct_task_msg(won_task)
            award_msg.tasks.append(task_msg)

            self.auctioneer.award_pub.publish(award_msg)

            rospy.logdebug("sending award message:\n{0}".format(pp.pformat(award_msg)))

            # Mark the task as awarded
            won_task.awarded = True

class AuctionSSI(Auction):
    """ A Sequential Single-Item auction.

    See:
    Koenig, Sven, et al. "The power of sequential single-item auctions for agent
    coordination." Proceedings of the National Conference on Artificial
    Intelligence. Vol. 21. No. 2. Menlo Park, CA; Cambridge, MA; London; AAAI
    Press; MIT Press; 1999, 2006.
    """
    mechanism_name = 'SSI'

    def __init__(self, auctioneer=None, tasks=None, auction_round=None):
        super(AuctionSSI, self).__init__(auctioneer, tasks, auction_round)

    def _construct_announcement_msg(self):
        announce_msg = multirobot_common.msg.AnnounceSensorSweep()
        announce_msg.mechanism = self.mechanism_name

        # Announce all messages at once
        for task in self.tasks:
            announce_msg.tasks.append(self._construct_task_msg(task))

        return announce_msg

    def announce(self, e):
        rospy.loginfo("(SSI) state: announce")

        while not self.auctioneer.team_members:
            rospy.logdebug("..waiting for team to be non-empty")
            time.sleep(1)

        announcement_msg = self._construct_announcement_msg()
        stamp(announcement_msg)
        self.auctioneer.announce_pub.publish(announcement_msg)

        rospy.logdebug("Announcement:\n{0}".format(pp.pformat(announcement_msg)))

        self.fsm.announced()

    def collect_bids(self, e):
        rospy.loginfo("(SSI) state: collect_bids")

        bids = self.auctioneer.bids[self.auction_round]

        # In SSI, we wait to receive as many bids as there are team members
        while len(bids) < len(self.auctioneer.team_members):
            time.sleep(0.2)

        self.fsm.bids_collected()

    def determine_winner(self, e):
        rospy.loginfo("(SSI) state: determine_winner")
        
        bids = self.auctioneer.bids[self.auction_round]

        rospy.logdebug("bids:\n{0}".format(pp.pformat(bids)))
        winner_id = None
        minimum_bid = None
        minimum_bid_task_id= None

        # We want to find the minimum bid on any task
        for robot_id in bids:
            for task_id in bids[robot_id]:
                bid_value = bids[robot_id][task_id]
                rospy.logdebug("determine_winner(): robot {0} bid {1} on task {2}".format(robot_id,
                                                                                          bid_value,
                                                                                          task_id))

                if minimum_bid is None or bid_value < minimum_bid:
                    winner_id = robot_id
                    minimum_bid = bid_value
                    minimum_bid_task_id = task_id

        rospy.loginfo("winner of task {0} is {1}".format(minimum_bid_task_id, winner_id))

        self.fsm.winner_determined(task_id=minimum_bid_task_id, winner_id=winner_id)

    def award(self, e):
        """ Construct and send an award message. """
        rospy.loginfo("(SSI) state: award")

        task_id = e.task_id
        task = self._get_task_by_id(task_id)
        winner_id = e.winner_id

        award_msg = multirobot_common.msg.TaskAward()
        award_msg.robot_id = e.winner_id

        task_msg = self._construct_task_msg(task)
        award_msg.tasks.append(task_msg)

        rospy.logdebug("sending award message:\n{0}".format(pp.pformat(award_msg)))

        stamp(award_msg)
        self.auctioneer.award_pub.publish(award_msg)

        # Mark the task as awarded
        task.awarded = True

class AuctionRR(Auction):

    mechanism_name = 'RR'

    def __init__(self, auctioneer=None, tasks=None, auction_round=None):
        super(AuctionRR, self).__init__(auctioneer, tasks, auction_round)

    def _construct_announcement_msg(self):
        announce_msg = multirobot_common.msg.AnnounceSensorSweep()
        announce_msg.mechanism = self.mechanism_name

        # Only announce one task (the first in the auctioneer's list)
        announce_msg.tasks.append(self._construct_task_msg(self.tasks[0]))

        return announce_msg

    def announce(self, e):
        rospy.loginfo("(RR) state: announce")
        rospy.loginfo("..skipping!")

        self.fsm.announced()

    def collect_bids(self, e):
        rospy.loginfo("(RR) state: collect_bids")
        rospy.loginfo("..skipping!")

        self.fsm.bids_collected()

    def determine_winner(self, e):
        rospy.loginfo("(RR) state: determine_winner")
        rospy.loginfo("..skipping!")

        self.fsm.winner_determined()

    def award(self, e):
        """ Construct and send an award message for each task. """
        rospy.loginfo("(RR) state: award")

        # A cycling iterator of team member names
        team_cycle = itertools.cycle(self.auctioneer.team_members)

        for task in self.tasks:
            award_msg = multirobot_common.msg.TaskAward()
            award_msg.robot_id = team_cycle.next()

            task_msg = self._construct_task_msg(task)
            award_msg.tasks.append(task_msg)

            rospy.logdebug("sending award message:\n{0}".format(pp.pformat(award_msg)))

            stamp(award_msg)
            self.auctioneer.award_pub.publish(award_msg)

            # Mark the task as awarded
            task.awarded = True

class Auctioneer:

    def __init__(self, mechanism=None, task_file=None):
        """
        Initialize some ROS stuff (topics to publish/subscribe) and our state machine.
        """

        # Initialize our node
        # Do we need a special name for the auctioneer (i.e., not "auctioneer")?
        node_name = 'auctioneer'
        rospy.loginfo("Starting node '{0}'...".format(node_name))
        rospy.init_node(node_name)

        # Topics we wish to subscribe to
        self.init_subscribers()

        # Topics we wish to publish
        self.init_publishers()

        # The rate at which we'll sleep while idle
        self.rate = rospy.Rate(RATE)

        # A list of (node) names of robot team members.
        self.team_members = []

        # A list of (node) names team members who have completed
        # all of their tasks.
        self.team_members_completed = []

        # We will use one mechanism per run (for now)
        #self.mechanism = mechanism
        self.mechanism = rospy.get_param('~mechanism')

        # Tasks are loaded from a configuration file, rather than
        # loaded dynamically (for now)
        #self.task_file = task_file
        self.task_file = rospy.get_param('~task_file')

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

        # Generate a unique, random experiment id
        self.experiment_id = str(uuid.uuid4())

        # Send a message to mark the beginning of the experiment
        begin_exp_msg = multirobot_common.msg.ExperimentEvent()
        begin_exp_msg.experiment_id = self.experiment_id
        begin_exp_msg.event = 'BEGIN_EXPERIMENT'
        stamp(begin_exp_msg)
        self.experiment_pub.publish(begin_exp_msg)

        # Start the state machine
        self.fsm.startup()

    def init_subscribers(self):
        rospy.loginfo('Initializing subscribers...')

        # Listen for bids on '/tasks/bid'
        self.bid_sub = rospy.Subscriber('/tasks/bid',
                                        multirobot_common.msg.TaskBid,
                                        self.on_bid_received)

        self.bid_sub = rospy.Subscriber('/tasks/status',
                                        multirobot_common.msg.TaskStatus,
                                        self.on_task_status)

        # For good measure...
        time.sleep(3)

    def init_publishers(self):

        # Announce experiment events on '/experiment'.
        # Importantly, 'BEGIN_ALLOCATION', 'END_ALLOCATION', and
        # 'BEGIN_EXECUTION'.
        self.experiment_pub = rospy.Publisher('/experiment',
                                              multirobot_common.msg.ExperimentEvent)

        # Announce tasks on '/tasks/announce'. For the moment we will only
        # announce sensor sweep tasks.
        self.announce_pub = rospy.Publisher('/tasks/announce',
                                            multirobot_common.msg.AnnounceSensorSweep)

        # Award tasks on '/tasks/award'
        self.award_pub = rospy.Publisher('/tasks/award',
                                         multirobot_common.msg.TaskAward)

        # For good measure...
        time.sleep(3)

    def load_tasks(self, data):
        rospy.loginfo("Loading tasks from {0}...".format(self.task_file))

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
            rospy.logerr("Can't open task file {0} for reading!".format(self.task_file))

        rospy.loginfo("Tasks:\n{0}".format(pp.pformat(self.tasks)))

        self.fsm.tasks_loaded()

    def identify_team(self, data):
        rospy.loginfo("Identifying team...")

        # In the 'rosnode' utility/module, _sub_rosnode_listnodes() returns a
        # newline-separated list of the names of all nodes in the graph.
        # See http://wiki.ros.org/rosnode
        node_list = rosnode._sub_rosnode_listnodes().split()

        # We're finding namespaces that look like the following pattern
        # (in parens):
        name_pat = re.compile('/(.*)/robot_controller')
        for node_name in node_list:
            m = name_pat.match(node_name)
            if m:
                teammate_name = m.group(1)
                rospy.loginfo("Adding {0} to team".format(teammate_name))
                self.team_members.append(teammate_name)

        rospy.loginfo("Team members: {0}".format(self.team_members))

        self.fsm.team_identified()

    def idle(self, data):
        rospy.loginfo("state: idle")

        if not self.tasks:
            rospy.loginfo("idling...")
            self.rate.sleep()
            self.fsm.no_tasks()

        # Transition to the "choose_mechanism" state
        self.fsm.have_tasks()

    def choose_mechanism(self, e):
        rospy.loginfo("state: choose_mechanism")

        # Send a message to mark the beginning of the allocation phase of
        # the experiment
        begin_alloc_msg = multirobot_common.msg.ExperimentEvent()
        begin_alloc_msg.experiment_id = self.experiment_id
        begin_alloc_msg.event = 'BEGIN_ALLOCATION'
        stamp(begin_alloc_msg)
        self.experiment_pub.publish(begin_alloc_msg)

        # For now, use the single mechanism given to us as a parameter
        rospy.loginfo("  {0}".format(self.mechanism))

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
            elif self.mechanism == 'PSI':
                auction_psi = AuctionPSI(self, unallocated, self.auction_round)
            elif self.mechanism == 'SSI':
                auction_ssi = AuctionSSI(self, unallocated, self.auction_round)
            elif self.mechanism == 'RR':
                auction_rr = AuctionRR(self, unallocated, self.auction_round)

        # At this point, we can (safely?) consider all tasks awarded

        # We are finished allocating tasks. Signal the end of the allocation
        # phase and the beginning of the execution phase.
        end_alloc_msg = multirobot_common.msg.ExperimentEvent()
        end_alloc_msg.experiment_id = self.experiment_id
        end_alloc_msg.event = 'END_ALLOCATION'
        stamp(end_alloc_msg)
        self.experiment_pub.publish(end_alloc_msg)

        begin_exec_msg = multirobot_common.msg.ExperimentEvent()
        begin_exec_msg.experiment_id = self.experiment_id
        begin_exec_msg.event = 'BEGIN_EXECUTION'
        stamp(begin_exec_msg)
        self.experiment_pub.publish(begin_exec_msg)

        self.fsm.no_tasks()

    def on_bid_received(self, bid_msg):
        task_id = bid_msg.task_id
        robot_id = bid_msg.robot_id
        bid = bid_msg.bid

        round_bids = self.bids[self.auction_round]
        
        if not round_bids[robot_id]:
            round_bids[robot_id] = {}

        round_bids[robot_id][task_id] = float(bid)

        rospy.loginfo("{0} bid {1} for task {2} in auction round {3}".format(
            robot_id, bid, task_id, self.auction_round))

    def on_task_status(self, status_msg):
        robot_id = status_msg.robot_id
        task_id = status_msg.task_id
        status = status_msg.status

        # We mainly want to keep track of robots that have
        # completed all of their tasks
        if status == 'ALL_TASKS_COMPLETE':
            rospy.loginfo("Received ALL_TASKS_COMPLETE from {0}".format(robot_id))
            self.team_members_completed.append(robot_id)

    def end_experiment(self, e):
        rospy.loginfo("state: end_experiment")

        # Don't end the experiment until all of our team members
        # have finished their tasks
        rospy.loginfo("  waiting for team to finish all of their tasks...")
        while Set(self.team_members) != Set(self.team_members_completed):
            time.sleep(0.2)
        
        rospy.loginfo("  ...experiment finished.")
        # Send a message to mark the end of the execution phase
        end_exec_msg = multirobot_common.msg.ExperimentEvent()
        end_exec_msg.experiment_id = self.experiment_id
        end_exec_msg.event = 'END_EXECUTION'
        stamp(end_exec_msg)
        self.experiment_pub.publish(end_exec_msg)

        # Send a message to mark the end of the experiment
        end_exp_msg = multirobot_common.msg.ExperimentEvent()
        end_exp_msg.experiment_id = self.experiment_id
        end_exp_msg.event = 'END_EXPERIMENT'
        stamp(end_exp_msg)
        self.experiment_pub.publish(end_exp_msg)

        rospy.loginfo("end experiment")

        # Instead of exiting, wait to be shut down from outside
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    # Exit on ctrl-C
    signal.signal(signal.SIGINT, on_sigint)

    try:
        argv = rospy.myargv(argv=sys.argv[1:])
        auc = Auctioneer(*argv)
    except rospy.ROSInterruptException:
        pass
