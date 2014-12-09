#!/usr/bin/env python

# Standard modules
import sys
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

        argv = rospy.myargv(argv=sys.argv[1:])
        print "arguments: {0}".format(argv)

        # Our robot name is the first arg to our program 
        robot_name = None
        if not argv:
            # a random ID with dashes and underscores removed
            robot_name = str(uuid.uuid1()).replace('-','').replace('_','')
        else:
            robot_name = argv[0]

        # What messages do we want to publish?
        # - Send bids to auctioneer
        # - Send goals to navigation stack
        # - Send collision resolution messages
#        self.pub = rospy.Publisher('robot_status', String, queue_size=10)

        # Initialize our node
        node_name = "rc_{0}".format(robot_name)
#        rospy.loginfo("Starting node '{0}'...".format(node_name))
        print("Starting node '{0}'...".format(node_name))
        rospy.init_node(node_name)

        # Subscribed topics
        self.init_subscribers()

        # Topics to publish
        self.init_publishers()

        # actionlib client; used to send goals to the navigation stack
#        ac_name = "ac_{0}".format(robot_name)
        ac_name = "move_base"
        print "Starting actionlib client '{0}'".format(ac_name)
        self.aclient = actionlib.SimpleActionClient(ac_name,
                                                    move_base_msgs.msg.MoveBaseAction)

        # Wait until the action server has started up
        self.aclient.wait_for_server()

        print "{0} connected.".format(ac_name)

        # Set up state machine
#        self._state = RCState.IDLE
        self._state = RCState.INIT_POSE

    def set_gound_truth_pose(self, data):
#        print 'Setting ground truth pose...'
        self.ground_truth_pose = data
#        print self.ground_truth_pose

    def init_subscribers(self):
        print 'Initializing subscribers...'

        # 'base_pose_ground_truth'
        print '  base_pose_ground_truth'
        self.ground_truth_sub = rospy.Subscriber('base_pose_ground_truth',
                                                 nav_msgs.msg.Odometry,
                                                 self.set_gound_truth_pose)

    def init_publishers(self):
        # 'initialpose'
        self.initial_pose_pub = rospy.Publisher('initialpose',
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

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = 40.0
        goal.target_pose.pose.position.y = 20.0
        goal.target_pose.pose.orientation.w = 0.9

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
                print "Idle..."
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
