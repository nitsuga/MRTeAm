#!/usr/bin/env python

"""robot_controller.py

This script (and class) controls the task-bidding and goal-seeking behavior of a robot.

Usage: robot_controller.py [robot_name] [goal-x-pos] [goal-y-pos]

 robot_name: the desired ROS node name of the robot. If none is given, then a
  random name is chosen using uuid.uuid1()

 goal-x-pos:
 goal-y-pos: coordinates of an initial goal that the robot will towards. These arguments
  are used for testing. Normally, goals will be awarded (via ROS messages) to the robot
  by an auctioneer agent.


The RobotController class uses a state machine to implement bidding and goal-seeking
behavior.

Eric Schneider <eric.schneider@liverpool.ac.uk>
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
from std_msgs.msg import String

## TODO:
# Import auction, collision avoidance message types
# Import Task class

# We'll sleep 1/RATE seconds in every pass of the idle loop.
RATE = 10

# Enumeration of states for our 
class RCState:
    IDLE      = 1
    INIT_POSE = 2
    SEND_GOAL = 3

class RobotController:

    def __init__(self, robot_name=None, goal_x=None, goal_y=None):
        """
        Initialize some ROS stuff (pub/sub, actionlib client) and also
        our state machine.
        """

        # List of tasks/goals that we have been awarded
        self.agenda = []

        # Node name
        if robot_name:
            self.robot_name = robot_name
        else:
            # a random ID with dashes and underscores removed
            self.robot_name = str(uuid.uuid1()).replace('-','').replace('_','')
        print "Robot name: {0}".format(self.robot_name)

        # Initial goal, if any
        self.goal_x = goal_x
        self.goal_y = goal_y
        print "Goal: ({0}, {1})".format(self.goal_x, self.goal_y)

        # Initialize our node
        node_name = "rc_{0}".format(self.robot_name)
        #rospy.loginfo("Starting node '{0}'...".format(node_name))
        print("Starting node '{0}'...".format(node_name))
        rospy.init_node(node_name)

        # Topics we wish to subscribe to
        self.init_subscribers()

        # Topics we wish to publish
        self.init_publishers()

        # actionlib client; used to send goals to the navigation stack
        ac_name = "/{0}/move_base".format(self.robot_name)
        print "Starting actionlib client '{0}'".format(ac_name)
        self.aclient = actionlib.SimpleActionClient(ac_name,
                                                    move_base_msgs.msg.MoveBaseAction)

        # Wait until the action server has started up
        self.aclient.wait_for_server()
        print "{0} connected.".format(ac_name)

        # Set up state machine
        #        self._state = RCState.IDLE
        #self._state = RCState.INIT_POSE
        self._state = RCState.SEND_GOAL

        self.fsm = Fysom( initial='running',
                           events=[
                              # Bidding on tasks/adding goals
                              ('task_announced', 'running', 'bid'),
                              ('bid_sent', 'bid', 'running'),
                              ('task_won', 'running', 'won'),
                              ('goal_added', 'won', 'running'),
                              # Choosing/sending goals
                              ('has_goals', 'running', 'choose_goal'),
                              ('goal_chosen', 'choose_goal', 'send_goal'),
                              ('goal_sent', 'send_goal', 'running'),
                              # Goal success/failure
                              ('goal_reached', 'running', 'goal_success'),
                              ('goal_failed', 'running', 'goal_failure'),
                              # Pause/resume
                              ('pause', 'running', 'paused'),
                              ('resume', ['paused', 'goal_success', 'goal_failure'], 'running')
                          ],
                        callbacks={
                            'onbid': self.bid,
                            'onwon': self.add_task }
                      )

    def bid(self, e):
        pass

    def add_task(self, e):
        pass

    def set_gound_truth_pose(self, data):
#        print 'Setting ground truth pose...'
        self.ground_truth_pose = data
#        print self.ground_truth_pose

    def init_subscribers(self):
        print 'Initializing subscribers...'

        # 'base_pose_ground_truth'
        print '  base_pose_ground_truth'
        self.ground_truth_sub = rospy.Subscriber("/{0}/base_pose_ground_truth".format(self.robot_name),
                                                 nav_msgs.msg.Odometry,
                                                 self.set_gound_truth_pose)

    def init_publishers(self):
        # 'initialpose'
        self.initial_pose_pub = rospy.Publisher("/{0}/initialpose".format(self.robot_name),
                                                geometry_msgs.msg.PoseWithCovarianceStamped)

    def send_initial_pose(self):

        # Wait until we get a ground truth pose
        rate = rospy.Rate(RATE)
        while not self.ground_truth_pose:
            rate.sleep()

        print 'Sending initialpose...'

        initpose = geometry_msgs.msg.PoseWithCovarianceStamped()
        initpose.header.frame_id = 'map'

        initpose.pose = self.ground_truth_pose.pose

        self.initial_pose_pub.publish(initpose)

        self._state = RCState.SEND_GOAL

    def send_goal(self):
        goal = move_base_msgs.msg.MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = float(self.goal_x)
        goal.target_pose.pose.position.y = float(self.goal_y)
        goal.target_pose.pose.orientation.w = 1.0

        print "Sending goal: {0}".format(goal)
        self.aclient.send_goal(goal)

        # Wait until the goal is reached
        #self.aclient.wait_for_result()

        self._state = RCState.IDLE

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

        rc = RobotController(*argv)
        rc.spin()
    except rospy.ROSInterruptException:
        pass
