#!/usr/bin/env python

"""robot_controller.py

This script (and class) controls the task-bidding and goal-seeking behavior of a robot.

Usage: robot_controller.py [robot_name] [goal-x-pos] [goal-y-pos]

 robot_name: the desired ROS node name of the robot. If none is given, then a
  random name is chosen.

 goal-x-pos:
 goal-y-pos: coordinates of an initial goal that the robot will towards. These arguments
  are used for testing. Normally, goals will be awarded (via ROS messages) to the robot
  by an auctioneer agent.


The RobotController class uses a state machine to implement bidding and goal-seeking
behavior.

Eric Schneider <eric.schneider@liverpool.ac.uk>
"""

# Standard Python modules
import math
import pprint
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
import nav_msgs.srv
import navfn.srv
import rospy
import tf.transformations
#from std_msgs.msg import String

import multirobot_common
import multirobot_common.msg

# We'll sleep 1/RATE seconds in every pass of the idle loop.
RATE = 10

pp = pprint.PrettyPrinter(indent=2)

class RobotController:

    def __init__(self, robot_name=None, goal_x=None, goal_y=None):
        """
        Initialize some ROS stuff (pub/sub, actionlib client) and also
        our state machine.
        """
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

        # The name of the planner service
        self.plan_srv_name = "/{0}/move_base_node/NavfnROS/make_plan".format(self.robot_name)

        # Our current estimated pose. This should be received the navigation
        # stack's amcl localization package. See on_amcl_pose_received().
        self.current_pose = None

        # For some mechanisms, in order to compute a bid value (path cost)
        # we need to compute a path from the position of the most-recently-awarded
        # task point, rather than from the robot's current position
        self.last_won_location = None

        # The rate at which we'll sleep while idle
        self.rate = rospy.Rate(RATE)

        # List of tasks that we have been awarded (multirobot_common.SensorSweepTask)
        self.agenda = []

        # Set up state machine.
        # See multirobot/docs/robot-controller.fsm.png
        self.fsm = Fysom(
            events=[
                ('startup', 'none', 'idle'),

                # Bidding on tasks/adding goals
                ('task_announced', 'idle', 'bid'),
                ('bid_sent', 'bid', 'idle'),
                ('bid_not_sent', 'bid', 'idle'),
                ('task_won', 'idle', 'won'),
                ('task_added', 'won', 'idle'),
                
                # Choosing/sending goals
                ('have_tasks', 'idle', 'choose_task'),
                ('goal_chosen', 'choose_task', 'send_goal'),
                ('goal_sent', 'send_goal', 'moving'),
                
                # Goal success/failure
                ('goal_reached', 'moving', 'task_success'),
                ('resume', 'task_success', 'idle'),
                ('goal_cannot_be_reached', 'moving', 'task_failure'),
                ('resume', 'task_failure', 'idle'),
                
                # Pause/resume
                ('pause', 'moving', 'paused'),
                ('resume', 'paused', 'moving'),
                
                # Collision avoidance
                ('collision_detected', 'moving', 'paused'),
                ('collision_detected', 'paused', 'resolve_collision'),
                ('collision_resolved', 'resolve_collision', 'moving'),
                
            ],
            callbacks={
                'onidle': self.idle,
                'onbid': self.bid,
                'onwon': self.won,
                'onchoose_task': self.choose_task,
                'onsend_goal': self.send_goal,
                'ontask_success': self.task_success,
                'ontask_failure': self.task_failure
            }
        )

        # Start the state machine
        self.fsm.startup()

    def init_subscribers(self):
        print 'Initializing subscribers...'

        # '/robot_<n>/amcl_pose'
        print "- /robot_{0}/amcl_pose".format(self.robot_name)
        self.amcl_pose_sub = rospy.Subscriber("/{0}/amcl_pose".format(self.robot_name),
                                                 geometry_msgs.msg.PoseWithCovarianceStamped,
                                                 self.on_amcl_pose_received)

        # '/tasks/announce'
        # We respond only to sensor sweep task announcements (for now)
        print '- /tasks/announce'
        self.announce_sub = rospy.Subscriber('/tasks/announce',
                                             multirobot_common.msg.AnnounceSensorSweep,
                                             self.on_task_announced)

        # '/tasks/award'
        print '- /tasks/award'
        self.announce_sub = rospy.Subscriber('/tasks/award',
                                             multirobot_common.msg.TaskAward,
                                             self.on_task_award)

    def init_publishers(self):
        # 'initialpose'
        self.initial_pose_pub = rospy.Publisher("/{0}/initialpose".format(self.robot_name),
                                                geometry_msgs.msg.PoseWithCovarianceStamped)

        # '/tasks/bid'
        self.bid_pub = rospy.Publisher('/tasks/bid',
                                       multirobot_common.msg.TaskBid)

    def _point_to_pose(self, point):
        pose_msg = geometry_msgs.msg.Pose()
        pose_msg.position = point
        pose_msg.orientation.w = 1.0
        
        return pose_msg

    def _pose_to_posestamped(self, pose, frame_id='map'):
        pose_stamped_msg = geometry_msgs.msg.PoseStamped()

        pose_stamped_msg.header.stamp = rospy.get_rostime()
        pose_stamped_msg.header.frame_id = frame_id
        pose_stamped_msg.pose = pose

        return pose_stamped_msg

    def _make_nav_plan_client(self, start, goal):

        #print("Waiting for service {0}".format(self.plan_srv_name))
        rospy.wait_for_service(self.plan_srv_name)
        #print("Service ready.")

        try:
            make_nav_plan = rospy.ServiceProxy(self.plan_srv_name,
                                               nav_msgs.srv.GetPlan)

            # We need to convert start and goal from type geometry_msgs.Pose
            # to type geometry_msgs.PoseStamped. tolerance (from the docs):
            # "If the goal is obstructed, how many meters the planner can 
            # relax the constraint in x and y before failing."
            start_stamped = self._pose_to_posestamped(start)
            goal_stamped = self._pose_to_posestamped(goal)

            req = nav_msgs.srv.GetPlanRequest()
            req.start = start_stamped
            req.goal = goal_stamped
            req.tolerance = 0.1

            resp = make_nav_plan( req )

            print("Got plan:")
            #pp.pprint(resp)
            
            return resp.plan.poses

        except rospy.ServiceException, e:
            print("Service call failed: {0}".format(e))

    def _normalize_angle(angle):
        res = angle
        while res > pi:
            res -= 2.0*pi
        while res < -pi:
            res += 2.0*pi
        return res

    def get_path_cost(self, start, goal):
        """
        Return the cost of a path from start to goal. The path is obtained
        from the navfn/MakeNavPlan service. The cost is the sum of distances
        between points in the path PLUS the angular changes (in randians)
        needed to re-orient from every point to its successor.

        This is taken from the alufr ROS package. See getPlanCost() in
        navstack_module.cpp at:
        https://code.google.com/p/alufr-ros-pkg/source/browse/.
        """

        print("Getting path from navfn/MakeNavPlan...")

        # 'path' is a list of objects of type geometry_msgs.PoseStamped
        path = self._make_nav_plan_client(start, goal)

        print("Got path")

        path_cost = 0.0
        from_pose = None
        for pose_stamped in path:
            if from_pose is None:
                from_pose = pose_stamped.pose
                continue
            else:
                to_pose = pose_stamped.pose
                
                pos_delta = math.hypot(to_pose.position.x - from_pose.position.x,
                                       to_pose.position.y - from_pose.position.y)

#                print("pos_delta: {0}".format(pos_delta))

                path_cost += pos_delta

                ## Adding rotational distance to the path cost sounds cool, but
                ## the path returned by make_plan doesn't seem to specify
                ## orientations in its poses.
                #yaw_to = tf.transformations.euler_from_quaternion(to_pose.orientation)[2]
                #yaw_from = tf.transformations.euler_from_quaternion(from_pose.orientation)[2]
                #yaw_delta = math.fabs(self._normalize_angle(yaw_to-yaw_from))
                #path_cost += yaw_delta

                from_pose = pose_stamped.pose

        return path_cost

    def on_task_announced(self, msg):
#        print("task announced:")
#        pp.pprint(msg)

        # Trigger the 'task_announced' fsm event, with the message as a parameter
        self.fsm.task_announced(msg=msg)

    def on_task_award(self, msg):
        # Trigger the 'task_won' fsm event, with the message as a parameter
        self.fsm.task_won(msg=msg)

    def _construct_bid_msg(self, task_id, robot_id, bid):
        bid_msg = multirobot_common.msg.TaskBid()
        bid_msg.task_id = task_id
        bid_msg.robot_id = robot_id
        bid_msg.bid = bid

        return bid_msg

    def bid(self, e):
        print("state: bid")
        announce_msg = e.msg

        # Our bid-from point is the location of our most recently won task,
        # if any. If we haven't won any tasks, bid from our current position.
        bid_from = None
        if self.last_won_location:
            # Convert last_won_location from a Point to a Pose
            bid_from = self._point_to_pose(self.last_won_pose)
        else:
            bid_from = self.current_pose

        # The mechanism determines the number and kind of bids we make
        if announce_msg.mechanism == 'OSI':
            print("mechanism == OSI")

            task_msg = announce_msg.tasks[0]
            path_cost = self.get_path_cost(bid_from,
                                           self._point_to_pose(task_msg.location))

            print("path_cost={0}".format(path_cost))

            bid_msg = self._construct_bid_msg(task_msg.task.task_id,
                                              self.robot_name,
                                              path_cost)

            print("bid_msg:")
            pp.pprint(bid_msg)

            self.bid_pub.publish(bid_msg)
            
        elif announce_msg.mechanism == 'SSI':
            pass
        elif announce_msg.mechanism == 'PSI':
            pass
        else:
            print("bid(): mechanism '{0}' not supported".format(self.mechanism))
            
        self.fsm.bid_sent()

    def won(self, e):
        award_msg = e.msg

        for task_msg in award_msg.tasks:
            new_task = multirobot_common.SensorSweepTask(str(task_msg.task.task_id),
                                                         float(task_msg.location.x),
                                                         float(task_msg.location.y))
            self.agenda.append(new_task)

        self.fsm.task_added()

    def choose_task(self, e):
        print("state: choose_task")

        goal_task = None
        for task in self.agenda:
            if not task.completed:
                goal_task = task
                break

        self.fsm.goal_chosen(goal_task=goal_task)

    def on_amcl_pose_received(self, amcl_pose_msg):
        # amcl_pose_msg is typed as geometry_msgs/PoseWithCovarianceStamped.
        # We'll just keep track of amcl_pose_msg.pose.pose, which is typed as
        # geometry_msgs/Pose
        self.current_pose = amcl_pose_msg.pose.pose


    def send_goal(self, e):
        print("state: send_goal")

        goal_task = e.goal_task
        
        goal_msg = move_base_msgs.msg.MoveBaseGoal()

        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = rospy.Time.now()

        goal_msg.target_pose.pose.position.x = float(goal_task.location.x)
        goal_msg.target_pose.pose.position.y = float(goal_task.location.y)
        goal_msg.target_pose.pose.orientation.w = 1.0

        print "Sending goal: {0}".format(goal_msg)
        self.aclient.send_goal(goal_msg)

        # Wait until the goal is reached
        self.aclient.wait_for_result()

        self.fsm.goal_reached(goal_task=goal_task)

    def task_success(self, e):
        print("state: task_success")

        e.goal_task.completed = True

        self.fsm.resume()


    def task_failure(self, e):
        print("state: task_failure")

        e.goal_task.completed = False

        self.fsm.resume()
    

    def idle(self, e):
        print("state: idle")

        while not self.agenda:
#            print("idle..")
            self.rate.sleep()

        # We've been awarded least one task while idling
        self.fsm.have_tasks()

if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv=sys.argv[1:])
        print "arguments: {0}".format(argv)
        rc = RobotController(*argv)
    except rospy.ROSInterruptException:
        pass
