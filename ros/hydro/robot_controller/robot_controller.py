#!/usr/bin/env python

# Standard modules
import sys
import time
import uuid

# ROS modules
import actionlib
import geometry_msgs.msg
import move_base_msgs.msg
import nav_msgs.msg
import rospy
from std_msgs.msg import String

# Rate (Hz) at which we do work
RATE = 10

class RCState:
    IDLE      = 1
    INIT_POSE = 2
    SEND_GOAL = 3

class RobotController:

    def __init__(self):

#        time.sleep(5)
        
        argv = rospy.myargv(argv=sys.argv[1:])
        print "arguments: {0}".format(argv)

        # Our robot name is the first arg to our program 
        self.robot_name = None
        if not argv:
            # a random ID with dashes and underscores removed
            self.robot_name = str(uuid.uuid1()).replace('-','').replace('_','')
        else:
            self.robot_name = argv[0]

        print "Robot name: {0}".format(self.robot_name)

        self.goal_x = float(argv[1])
        self.goal_y = float(argv[2])

        print "Goal: ({0}, {1})".format(self.goal_x, self.goal_y)

        # What messages do we want to publish?
        # - Send bids to auctioneer
        # - Send goals to navigation stack
        # - Send collision resolution messages
#        self.pub = rospy.Publisher('robot_status', String, queue_size=10)

        # Initialize our node
        node_name = "rc_{0}".format(self.robot_name)
#        rospy.loginfo("Starting node '{0}'...".format(node_name))
        print("Starting node '{0}'...".format(node_name))
        rospy.init_node(node_name)

        # Subscribed topics
        self.init_subscribers()

        # Topics to publish
        self.init_publishers()

        # actionlib client; used to send goals to the navigation stack
#        ac_name = "ac_{0}".format(self.robot_name)
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
        rc = RobotController()
        rc.spin()
    except rospy.ROSInterruptException:
        pass
