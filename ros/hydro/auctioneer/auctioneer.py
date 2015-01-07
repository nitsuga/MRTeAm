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
import rospy
import unique_id

import multirobot_common.msg

## TODO:
# Import Task class

# We'll sleep 1/RATE seconds in every pass of the idle loop.
RATE = 10

class Auctioneer:

    def __init__(self, mechanism=None):
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

        # See multirobot/docs/auctioneer-fsm.png
        self.fsm = Fysom( initial='load_tasks',
                          events=[
                              ('tasks_loaded', 'load_tasks', 'identify_team'),
                              ('team_identified', 'identify_team', 'idle'),
                              ('no_tasks', '*', 'idle'),
                              ('have_tasks', 'idle', 'choose_mechanism'),
                              
                              ('OSI', 'choose_mechanism', 'announce'),
                              ('PSI', 'choose_mechanism', 'announce'),
                              ('SSI', 'choose_mechanism', 'announce'),
                              ('RR', 'choose_mechanism', 'determine_winner'),
                              
                              ('announced', 'announce', 'collect_bids'),
                              ('bids_collected', 'collect_bids', 'determine_winner'),
                              ('winner_determined', 'determine_winner', 'award')
                              ('have_tasks', 'award', 'choose_mechanism')
                          ],
                          callbacks={}
                      )

    def init_subscribers(self):
        print 'Initializing subscribers...'

        # Listen for bids on '/tasks/bid'
        self.bid_sub = rospy.Subscriber('/tasks/bid',
                                        multirobot_common.msg.TaskBid,
                                        self.on_bid)

    def init_publishers(self):

        # Announce tasks on '/tasks/announce'. For the moment we will only
        # announce sensor sweep tasks.
        self.annouce_pub = rospy.Publisher('/tasks/announce',
                                           multirobot_common.msg.AnnounceSensorSweep)

        # Award tasks on '/tasks/award'
        self.award_pub = rospy.Publisher('/tasks/award',
                                         multirobot_common.msg.TaskAward)

    def on_bid(self, data):
        # TODO: implement!
        pass

    def spin(self):

        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            # Do something
            if self._state == RCState.IDLE:
                #print "Idle..."
                pass
            elif self._state == RCState.INIT_POSE:
                self.send_initial_pose()
            elif self._state == RCState.SEND_GOAL:
                self.send_goal()

            rate.sleep()

if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv=sys.argv[1:])
        print "arguments: {0}".format(argv)

        rc = Auctioneer(*argv)
        rc.spin()
    except rospy.ROSInterruptException:
        pass
