#!/usr/bin/env python

"""robot_controller.py

This script (and class) controls the task-bidding and goal-seeking behavior of a
robot.

Usage: robot_controller.py [robot_name] [goal-x-pos] [goal-y-pos]

 robot_name: the desired ROS node name of the robot. If none is given, then a
  random name is chosen.

 goal-x-pos:
 goal-y-pos: coordinates of an initial goal that the robot will towards. These
  arguments are used for testing. Normally, goals will be awarded (via ROS
  messages) to the robot by an auctioneer agent.


The RobotController class uses a state machine to implement bidding and goal-
seeking behavior.

Eric Schneider <eric.schneider@liverpool.ac.uk>
"""

# Standard Python modules
from collections import defaultdict
import math
import pprint
import sys
import time
import uuid

# Fysom state machine
from fysom import Fysom

# ROS modules
import actionlib
from actionlib import SimpleActionClient
import actionlib_msgs.msg
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
RATE = 100

# If another robot is with this distance (in meters) and field of view (in
# radians), we're in danger of colliding with it
DANGER_ZONE_FOV = math.pi / 2
DANGER_ZONE_DIST = 0.8

pp = pprint.PrettyPrinter(indent=2)

def stamp(msg):
    """ Set the timestamp of a message to the current wall-clock time."""
    rospy.rostime.switch_to_wallclock()
    msg.header.stamp = rospy.rostime.get_rostime()


class Collision(object):
    def __init__(self):
        pass

def in_danger_zone(my_pose, other_pose):

    in_danger = False

    if not my_pose or not other_pose:
        return in_danger

    # Angle between my_pose and the positive y-axis

    # my_pose orientation quaternion
    my_q = (my_pose.orientation.x,
            my_pose.orientation.y,
            my_pose.orientation.z,
            my_pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(my_q)
    my_yaw = euler[2]
    rospy.logdebug("my_yaw == {0}".format(my_yaw))
    d_theta = (math.pi / 2) - my_yaw
    rospy.logdebug("d_theta == {0}".format(d_theta))

    # Translate the treasure's position, pretending the robot is at the origin
    trans_x = other_pose.position.x - my_pose.position.x
    trans_y = other_pose.position.y - my_pose.position.y
    rospy.logdebug("trans_x=={0}, trans_y=={1}".format(trans_x, trans_y))


    # Rotate the translated point by d_theta
    rot_x = (trans_x * math.cos(d_theta)) - (trans_y * math.sin(d_theta))
    rot_y = (trans_x * math.sin(d_theta)) + (trans_y * math.cos(d_theta))
    rospy.logdebug("rot_x=={0}, rot_y=={1}".format(rot_x, rot_y))

    other_heading = math.atan2(rot_x, rot_y)
    rospy.logdebug("other_pos heading is {0}".format(other_heading))

    other_dist = math.hypot(other_pose.position.x - my_pose.position.x,
                            other_pose.position.y - my_pose.position.y)

    in_fov = False
    if (other_heading >= -(DANGER_ZONE_FOV/2) and
        other_heading <= (DANGER_ZONE_FOV/2)):
        in_fov = True

    rospy.logdebug("in_fov=={0}".format(in_fov))

    in_dist = False
    if other_dist <= DANGER_ZONE_DIST:
        in_dist = True
    rospy.logdebug("in_dist=={0}".format(in_dist))

    if in_fov and in_dist:
        in_danger = True
    rospy.logdebug("in_danger=={0}".format(in_danger))

    return in_danger

class RobotController:
    """ Controls robot behavior. """
    def __init__(self, robot_name=None):
        """
        Initialize some ROS stuff (pub/sub, actionlib client) and also
        our state machine.
        """
        # Node name
        if robot_name:
            self.robot_name = robot_name
        else:
            # a random ID with dashes and underscores removed
            self.robot_name = str(uuid.uuid1())
            self.robot_name = self.robot_name.replace('-', '').replace('_', '')
        rospy.loginfo("Robot name: {0}".format(self.robot_name))

        # Our current estimated pose. This should be received the navigation
        # stack's amcl localization package. See on_my_pose_received().
        self.current_pose = None

        # Our current task (multirobot_common.SensorSweepTask)
        self.current_task = None

        self.other_poses = defaultdict(geometry_msgs.msg.Pose)
        self.amcl_pose_subs = {}

        # Keep track of potential collisions with other robots
        self.collisions = defaultdict(Collision)

        # For some mechanisms, in order to compute a bid value (path cost)
        # we need to compute a path from the position of the most-recently-
        # awarded task point, rather than from the robot's current position.
        self.last_won_location = None

        # List of tasks that we have been awarded (SensorSweepTask)
        self.agenda = []

        # Unless true, we may only bid on tasks but NOT begin executing tasks
        self.ok_to_execute = False

        # Initialize our node
        node_name = 'robot_controller'
        rospy.loginfo("Starting node '{0}'...".format(node_name))
        rospy.init_node(node_name)

        # The rate at which we'll sleep while idle
        self.rate = rospy.Rate(RATE)

        # Topics we wish to subscribe to
        self.init_subscribers()

        # Topics we wish to publish
        self.init_publishers()

        # actionlib client; used to send goals to the navigation stack
        ac_name = "/{0}/move_base".format(self.robot_name)
        rospy.loginfo("Starting actionlib client '{0}'".format(ac_name))
        self.aclient = SimpleActionClient(ac_name,
                                          move_base_msgs.msg.MoveBaseAction)

        # Wait until the action server has started up
        self.aclient.wait_for_server()
        rospy.loginfo("{0} connected.".format(ac_name))

        # The name of the planner service
        self.plan_srv_name = "/{0}/move_base_node/NavfnROS/make_plan".format(
            self.robot_name)

        # Set up state machine.
        # See multirobot/docs/robot-controller.fsm.png
        self.fsm = Fysom(
            events=[
                ('startup', 'none', 'idle'),

                # Choosing/sending goals
                ('have_tasks', 'idle', 'choose_task'),
                ('no_tasks', 'idle', 'shutdown'),
                #('no_tasks', 'idle', 'idle'),
                ('goal_chosen', 'choose_task', 'send_goal'),
                #('no_tasks', 'choose_task', 'idle'),
                ('goal_sent', 'send_goal', 'moving'),

                # Goal success/failure
                ('goal_reached', '*', 'task_success'),
                ('resume', 'task_success', 'idle'),
                ('goal_not_reached', '*', 'task_failure'),
                ('resume', 'task_failure', 'idle'),

                # Pause/resume
                ('pause', 'moving', 'paused'),
                ('resume', 'paused', 'moving'),

                # Collision avoidance
                ('collision_detected', 'moving', 'resolve_collision'),
                #('collision_detected', 'paused', 'resolve_collision'),
                ('collision_resolved', 'resolve_collision', 'moving'),
            ],
            callbacks={
                'onidle': self.idle,
                #'onbid': self.bid,
                #'onwon': self.won,
                'onchoose_task': self.choose_task,
                'onsend_goal': self.send_goal,
                'onmoving': self.moving,
                'ontask_success': self.task_success,
                'ontask_failure': self.task_failure,
                'onshutdown': self.shutdown
            }
        )

        # Start the state machine
        self.fsm.startup()

    def init_subscribers(self):
        rospy.loginfo('Initializing subscribers...')

        # '/experiment'
        self.experiment_sub = rospy.Subscriber(
            '/experiment',
            multirobot_common.msg.ExperimentEvent,
            self.on_experiment_event_received)
        rospy.loginfo('subscribed to /experiment')

        # '/robot_<n>/amcl_pose'
        for robot_num in range(1, 4):
            r_name = "robot_{0}".format(robot_num)

            callback = None
            if r_name == self.robot_name:
                callback = self.on_my_pose_received
            else:
                callback = self.on_teammate_pose_received

            self.amcl_pose_subs[r_name] = rospy.Subscriber(
                "/{0}/amcl_pose".format(r_name),
                geometry_msgs.msg.PoseWithCovarianceStamped,
                callback, callback_args=r_name)
            rospy.loginfo("subscribed to /{0}/amcl_pose".format(r_name))

        # '/tasks/announce'
        # We respond only to sensor sweep task announcements (for now)
        self.announce_sub = rospy.Subscriber('/tasks/announce',
                                             multirobot_common.msg.AnnounceSensorSweep,
                                             self.bid)
        rospy.loginfo('subscribed to /tasks/announce')

        # '/tasks/award'
        self.announce_sub = rospy.Subscriber('/tasks/award',
                                             multirobot_common.msg.TaskAward,
                                             self.won)
        rospy.loginfo('subscribed to /tasks/award')

        # For good measure...
        time.sleep(3)

    def init_publishers(self):
        # '/tasks/bid'
        self.bid_pub = rospy.Publisher('/tasks/bid',
                                       multirobot_common.msg.TaskBid)
        rospy.loginfo('publishing on /tasks/bid')

        # Announce task events on '/tasks/status':
        # 'BEGIN', 'PAUSE', 'RESUME', 'SUCCESS', 'FAILURE', 'ALL_TASKS_COMPLETE'
        self.task_status_pub = rospy.Publisher('/tasks/status',
                                               multirobot_common.msg.TaskStatus)
        rospy.loginfo('publishing on /tasks/status')

        # For good measure...
        time.sleep(3)

    def _point_to_point_msg(self, point):
        """Convert a multirobot_common.Point to a geometry_msgs.msg.Point"""
        point_msg = geometry_msgs.msg.Point()
        point_msg.x = point.x
        point_msg.y = point.y
        point_msg.z = point.z

        return point_msg

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

        #rospy.logdebug("Waiting for service {0}".format(self.plan_srv_name))
        rospy.wait_for_service(self.plan_srv_name)
        #rospy.logdebug("Service ready.")

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

            resp = make_nav_plan(req)

            rospy.logdebug("Got plan:\n{0}".format(pp.pformat(resp)))

            return resp.plan.poses

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {0}".format(e))

    def _normalize_angle(self, angle):
        res = angle
        while res > math.pi:
            res -= 2.0 * math.pi
        while res < -pi:
            res += 2.0 * math.pi
        return res

    def handle_collision(self, other_name, other_pose):
        if in_danger_zone(self.current_pose, other_pose):
            rospy.loginfo("In danger of colliding with {0}".format(other_name))

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

        rospy.logdebug('Getting path from navfn/MakeNavPlan...')

        # 'path' is a list of objects of type geometry_msgs.PoseStamped
        path = self._make_nav_plan_client(start, goal)

        rospy.logdebug('...got path')

        path_cost = 0.0
        from_pose = None
        for pose_stamped in path:
            if from_pose is None:
                from_pose = pose_stamped.pose
                continue
            else:
                to_pose = pose_stamped.pose

                pos_delta = math.hypot(
                    to_pose.position.x - from_pose.position.x,
                    to_pose.position.y - from_pose.position.y)

                rospy.logdebug("pos_delta: {0}".format(pos_delta))

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

    def _cumulative_cost(self, start, tasks=None, greedy=True):
        """
        Get the cumulative cost of performing all tasks (visiting task points)
        from start. If greedy==True, tasks are ordered to minimize the cost of
        each hop (i.e., the next closest task is chosen). Otherwise, the
        original list order is used.
        """
        cumulative_cost = 0.0

        if greedy:
            #rospy.logdebug("_cumulative_cost(), tasks:\n{0}".format(pp.pformat(tasks)))

            tasks_copy = tasks[:]
            reordered_tasks = []

            from_pose = start
            while tasks_copy:
                min_cost = None
                min_cost_task = None
                for task in tasks_copy:
                    to_pose = self._point_to_pose(task.location)
                    cost = self.get_path_cost(from_pose, to_pose)

                    rospy.logdebug("cost from {0} to {1} is {2}".format(pp.pformat(from_pose),
                                                                        pp.pformat(to_pose),
                                                                        cost))

                    if min_cost is None or cost < min_cost:
                        min_cost = cost
                        min_cost_task = task

                reordered_tasks.append(min_cost_task)
                tasks_copy.remove(min_cost_task)
                from_pose = self._point_to_pose(min_cost_task.location)

            tasks = reordered_tasks
            rospy.logdebug("tasks=reordered_tasks={0}".format(pp.pformat(tasks)))

        from_pose = start
        for task in tasks:
            to_pose = self._point_to_pose(task.location)
            cost = self.get_path_cost(from_pose, to_pose)
            cumulative_cost += cost
            from_pose = to_pose

        final_pose = None
        if tasks:
            final_pose = self._point_to_pose(tasks[-1].location)
        else:
            final_pose = start

        return (cumulative_cost, final_pose)

    def on_task_announced(self, msg):
        # Trigger the 'task_announced' fsm event, with the message as a parameter
        rospy.logdebug("task announced:\n{0}".format(pp.pformat(msg)))
        self.fsm.task_announced(msg=msg)

    def on_task_award(self, msg):
        # Trigger the 'task_won' fsm event, with the message as a parameter
        rospy.logdebug("task won:\n{0}".format(pp.pformat(msg)))
        self.fsm.task_won(msg=msg)

    def _construct_bid_msg(self, task_id, robot_id, bid):
        bid_msg = multirobot_common.msg.TaskBid()
        bid_msg.task_id = task_id
        bid_msg.robot_id = robot_id
        bid_msg.bid = bid

        return bid_msg

    def bid(self, announce_msg):
        rospy.loginfo('bid()')

        # Our bid-from point is the location of our most recently won task,
        # if any. If we haven't won any tasks, bid from our current position.
        # bid_from = None
        # if self.last_won_location:
        #     # Convert last_won_location from a Point to a Pose
        #     bid_from = self._point_to_pose(self.last_won_location)
        # else:
        #     bid_from = self.current_pose

        # The mechanism determines the number and kind of bids we make
        if announce_msg.mechanism == 'OSI':
            rospy.loginfo("mechanism == OSI")

            task_msg = announce_msg.tasks[0]

            # path_cost = self.get_path_cost(bid_from,
            #                                self._point_to_pose(task_msg.location))


            # Get the cumulative cost of all tasks in out agenda so far
            (c_cost, bid_from) = self._cumulative_cost(self.current_pose,
                                                       self.agenda,
                                                       greedy=True)

            rospy.loginfo("({0}) cumulative cost: {1}".format(self.robot_name,
                                                              c_cost))

            new_task_cost = self.get_path_cost(bid_from,
                                               self._point_to_pose(task_msg.location))

            path_cost = c_cost + new_task_cost

            rospy.loginfo("({0}) path_cost to task {1}: {2}".format(
                self.robot_name,
                task_msg.task.task_id,
                path_cost))

            bid_msg = self._construct_bid_msg(task_msg.task.task_id,
                                              self.robot_name,
                                              path_cost)
            stamp(bid_msg)
            rospy.loginfo("bid_msg:\n{0}".format(pp.pformat(bid_msg)))
            self.bid_pub.publish(bid_msg)

        elif announce_msg.mechanism == 'PSI':
            rospy.loginfo("mechanism == PSI")

            # In PSI we always calculate bids (path costs) from our current
            # position
            bid_from = self.current_pose

            for task_msg in announce_msg.tasks:
                path_cost = self.get_path_cost(bid_from,
                                               self._point_to_pose(task_msg.location))

                rospy.loginfo("({0}) path_cost to task {1}: {2}".format(
                    self.robot_name,
                    task_msg.task.task_id,
                    path_cost))

                bid_msg = self._construct_bid_msg(task_msg.task.task_id,
                                                  self.robot_name,
                                                  path_cost)

                stamp(bid_msg)
                rospy.logdebug("bid_msg:\n{0}".format(pp.pformat(bid_msg)))
                self.bid_pub.publish(bid_msg)

        elif announce_msg.mechanism == 'SSI':
            rospy.loginfo("mechanism == SSI")

            # Get the cumulative cost of all tasks in out agenda so far
            (c_cost, bid_from) = self._cumulative_cost(self.current_pose,
                                                       self.agenda,
                                                       greedy=True)

            rospy.loginfo("({0}) cumulative cost: {1}".format(self.robot_name,
                                                              c_cost))
            minimum_cost = None
            minimum_cost_task_id = None

            for task_msg in announce_msg.tasks:
                #path_cost = self.get_path_cost(bid_from,
                #                               self._point_to_pose(task_msg.location))

                new_task_cost = self.get_path_cost(
                    bid_from,
                    self._point_to_pose(task_msg.location))

                path_cost = c_cost + new_task_cost

                rospy.loginfo("({0}) path_cost to task {1}: {2}".format(
                    self.robot_name,
                    task_msg.task.task_id,
                    path_cost))

                if minimum_cost is None or path_cost < minimum_cost:
                    minimum_cost = path_cost
                    minimum_cost_task_id = task_msg.task.task_id

            rospy.loginfo("({0}) minimum_cost={1} to task {2}".format(
                self.robot_name,
                minimum_cost,
                minimum_cost_task_id))

            bid_msg = self._construct_bid_msg(minimum_cost_task_id,
                                              self.robot_name,
                                              minimum_cost)

            stamp(bid_msg)
            rospy.logdebug("bid_msg:\n{0}".format(pp.pformat(bid_msg)))
            self.bid_pub.publish(bid_msg)

        else:
            rospy.logerr("bid(): mechanism '{0}' not supported".format(self.mechanism))

    def won(self, award_msg):
        rospy.loginfo("won()")

        # Make sure that this robot won the task
        if award_msg.robot_id != self.robot_name:
            return

        for task_msg in award_msg.tasks:
            rospy.loginfo("won task {0}".format(task_msg.task.task_id))
            new_task = multirobot_common.SensorSweepTask(str(task_msg.task.task_id),
                                                         float(task_msg.location.x),
                                                         float(task_msg.location.y))
            self.agenda.append(new_task)
            self.last_won_location = task_msg.location

    def choose_task(self, e):
        rospy.loginfo("state: choose_task")

        # We have two ways to choose the next task from our agenda:
        # 1. ("non-greedy") Choose the first task that hasn't been completed yet
        # 2. ("greedy")     Choose the next closest task

        greedy_selection = True

        # "non-greedy"
        goal_task = None
        min_uncompleted_dist = None
        for task in self.agenda:
            if not task.completed:
                if greedy_selection:
                    from_pose = self.current_pose
                    to_pose = self._point_to_pose(self._point_to_point_msg(task.location))
                    path_cost = self.get_path_cost(from_pose, to_pose)

                    if not min_uncompleted_dist or path_cost < min_uncompleted_dist:
                        goal_task = task
                        min_uncompleted_dist = path_cost

                else:
                    goal_task = task
                    break

        self.current_task = goal_task

        # Sanity check
        if not self.current_task:
            rospy.loginfo("event: no_tasks")
            self.fsm.no_tasks()

        rospy.loginfo("event: goal_chosen")
        self.fsm.goal_chosen()

    def on_my_pose_received(self, amcl_pose_msg, r_name):
        # amcl_pose_msg is typed as geometry_msgs/PoseWithCovarianceStamped.
        # We'll just keep track of amcl_pose_msg.pose.pose, which is typed as
        # geometry_msgs/Pose
        self.current_pose = amcl_pose_msg.pose.pose

    def on_teammate_pose_received(self, amcl_pose_msg, r_name):

        # self.fsm might not be initialized yet
        try:
            if self.fsm:
                pass
        except AttributeError:
            return

        # amcl_pose_msg is typed as geometry_msgs/PoseWithCovarianceStamped.
        # We'll just keep track of amcl_pose_msg.pose.pose, which is typed as
        # geometry_msgs/Pose
        other_pose = amcl_pose_msg.pose.pose
        self.other_poses[r_name] = other_pose

        if in_danger_zone(self.current_pose, other_pose):
            rospy.loginfo("In danger of colliding with {0}".format(r_name))
            if self.fsm.current == 'moving':
                self.fsm.collision_detected()
        else:
            if self.fsm.current == 'resolve_collision':
                self.fsm.collision_resolved()

        #self.handle_collision(r_name, other_pose)

        #rospy.loginfo("Got {0}'s pose:\n{1}".format(r_name, pp.pformat(self.other_poses[r_name])))

    def on_experiment_event_received(self, event_msg):
        if event_msg.event == 'BEGIN_EXECUTION':
            self.ok_to_execute = True
        #elif event_msg.event == 'END_EXPERIMENT':
        #    self.shutdown(None)

    # def goal_done_cb(self, term_state, result):
    #     rospy.loginfo("goal_done_cb(): term_state=={0}, result=={1}".format(term_state,
    #                                                                         result))
    #     if term_state == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
    #         self.fsm.goal_reached()
    #     else:
    #         self.fsm.goal_not_reached()

    def send_goal(self, e):
        rospy.loginfo("state: send_goal")

        goal_task = self.current_task

        # Send a message to mark the beginning of this task's execution
        begin_task_msg = multirobot_common.msg.TaskStatus()
        begin_task_msg.robot_id = self.robot_name
        begin_task_msg.task_id = goal_task.task_id
        begin_task_msg.status = 'BEGIN'
        stamp(begin_task_msg)
        self.task_status_pub.publish(begin_task_msg)

        # Construct the goal message to send to the move_base service
        goal_msg = move_base_msgs.msg.MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = rospy.Time.now()

        goal_msg.target_pose.pose.position.x = float(goal_task.location.x)
        goal_msg.target_pose.pose.position.y = float(goal_task.location.y)
        goal_msg.target_pose.pose.orientation.z = 1.0
        goal_msg.target_pose.pose.orientation.w = 0.0

        rospy.loginfo("Sending goal: {0}".format(pp.pformat(goal_msg)))
        #self.aclient.send_goal(goal_msg, done_cb=self.goal_done_cb)
        self.aclient.send_goal(goal_msg)

        self.fsm.goal_sent()

    def moving(self, e):
        rospy.loginfo("state: moving")

        # Wait until the goal is reached
        self.aclient.wait_for_result()

        # The actionlib client will send a "goal reached" message, triggering
        # goal_done_cb() to change the current state. Wait here until then.
        #while self.fsm.current == 'moving':
        #    self.rate.sleep()

        self.fsm.goal_reached()

    def task_success(self, e):
        rospy.loginfo("state: task_success")

        goal_task = self.current_task

        # Send a message to mark the end of this task's execution
        end_task_msg = multirobot_common.msg.TaskStatus()
        end_task_msg.robot_id = self.robot_name
        end_task_msg.task_id = goal_task.task_id
        end_task_msg.status = 'SUCCESS'
        stamp(end_task_msg)
        self.task_status_pub.publish(end_task_msg)

        goal_task.completed = True
        self.current_task = None

        self.fsm.resume()

    def task_failure(self, e):
        rospy.loginfo("state: task_failure")

        goal_task = self.current_task

        # Send a message to mark the end of this task's execution
        end_task_msg = multirobot_common.msg.TaskStatus()
        end_task_msg.robot_id = self.robot_name
        end_task_msg.task_id = goal_task.task_id
        end_task_msg.status = 'FAILURE'
        stamp(end_task_msg)
        self.task_status_pub.publish(end_task_msg)

        goal_task.completed = False
        self.current_task = None

        self.fsm.resume()

    def idle(self, e):
        rospy.loginfo("state: idle")

        # We idle here unless two conditions are true:
        # 1. We have tasks in our agenda
        # 2. self.ok_to_execute == True
        while not self.agenda or not self.ok_to_execute:
            self.rate.sleep()

        have_tasks = False
        for task in self.agenda:
            if not task.completed:
                have_tasks = True
                break

        if have_tasks:
            self.fsm.have_tasks()
        else:
            # The allocation and execution phases are finished.

            # Send a message to mark the completion of all tasks
            all_complete_msg = multirobot_common.msg.TaskStatus()
            all_complete_msg.robot_id = self.robot_name
            all_complete_msg.task_id = '*'
            all_complete_msg.status = 'ALL_TASKS_COMPLETE'
            stamp(all_complete_msg)
            self.task_status_pub.publish(all_complete_msg)

            self.fsm.no_tasks()

    def shutdown(self, e):
        # Do any cleanup here before shutting down

        rospy.loginfo("{0} shutting down...".format(self.robot_name))

        # Instead of exiting, wait to be shut down from outside
        #sys.exit(0)
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':

    try:
        argv = rospy.myargv(argv=sys.argv[1:])
        #print "arguments: {0}".format(argv)
        rc = RobotController(*argv)
        #print("rc final state: {0}".format(rc.fsm.current))

        while not rospy.is_shutdown():
            rc.rate.sleep()

    except rospy.ROSInterruptException:
        pass

    print('######## robot_controller exiting ########')
