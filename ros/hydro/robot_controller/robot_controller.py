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
RATE = 100

pp = pprint.PrettyPrinter(indent=2)

def stamp(msg):
    """ Set the timestamp of a message to the current wall-clock time."""
    rospy.rostime.switch_to_wallclock()
    msg.header.stamp = rospy.rostime.get_rostime()

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
        #node_name = "rc_{0}".format(self.robot_name)
        #node_name = "/{0}/robot_controller".format(self.robot_name)
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
        self.aclient = actionlib.SimpleActionClient(ac_name,
                                                    move_base_msgs.msg.MoveBaseAction)

        # Wait until the action server has started up
        self.aclient.wait_for_server()
        rospy.loginfo("{0} connected.".format(ac_name))

        # The name of the planner service
        self.plan_srv_name = "/{0}/move_base_node/NavfnROS/make_plan".format(self.robot_name)

        # Our current estimated pose. This should be received the navigation
        # stack's amcl localization package. See on_amcl_pose_received().
        self.current_pose = None

        # For some mechanisms, in order to compute a bid value (path cost)
        # we need to compute a path from the position of the most-recently-awarded
        # task point, rather than from the robot's current position
        self.last_won_location = None

        # List of tasks that we have been awarded (multirobot_common.SensorSweepTask)
        self.agenda = []

        # Unless true, we may only bid on tasks but NOT begin executing tasks
        self.ok_to_execute = False

        # Set up state machine.
        # See multirobot/docs/robot-controller.fsm.png
        self.fsm = Fysom(
            events=[
                ('startup', 'none', 'idle'),

                # Choosing/sending goals
                ('have_tasks', 'idle', 'choose_task'),
#                ('no_tasks', 'idle', 'shutdown'),
                ('no_tasks', 'idle', 'idle'),
                ('goal_chosen', 'choose_task', 'send_goal'),
                ('no_tasks', 'choose_task', 'idle'),
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
        self.experiment_sub = rospy.Subscriber('/experiment',
                                               multirobot_common.msg.ExperimentEvent,
                                               self.on_experiment_event_received)
        rospy.loginfo('subscribed to /experiment')

        # '/robot_<n>/amcl_pose'
        self.amcl_pose_sub = rospy.Subscriber("/{0}/amcl_pose".format(self.robot_name),
                                                 geometry_msgs.msg.PoseWithCovarianceStamped,
                                                 self.on_amcl_pose_received)
        rospy.loginfo("subscribed to /robot_{0}/amcl_pose".format(self.robot_name))

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

            resp = make_nav_plan( req )

            rospy.logdebug("Got plan:\n{0}".format(pp.pformat(resp)))
            
            return resp.plan.poses

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {0}".format(e))

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

    def _cumulative_cost(self, start, tasks=[], greedy=True):
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
        rospy.loginfo('state: bid')

        # Our bid-from point is the location of our most recently won task,
        # if any. If we haven't won any tasks, bid from our current position.
        bid_from = None
        if self.last_won_location:
            # Convert last_won_location from a Point to a Pose
            bid_from = self._point_to_pose(self.last_won_location)
        else:
            bid_from = self.current_pose

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

            new_task_cost = self.get_path_cost(bid_from,
                                               self._point_to_pose(task_msg.location))

            path_cost = c_cost + new_task_cost

            print("path_cost={0}".format(path_cost))

            bid_msg = self._construct_bid_msg(task_msg.task.task_id,
                                              self.robot_name,
                                              path_cost)

            rospy.loginfo("bid_msg:\n{0}".format(pp.pformat(bid_msg)))

            stamp(bid_msg)
            self.bid_pub.publish(bid_msg)
            
        elif announce_msg.mechanism == 'PSI':
            rospy.loginfo("mechanism == PSI")

            # In PSI we always calculate bids (path costs) from our current
            # position
            bid_from = self.current_pose

            for task_msg in announce_msg.tasks:                
                path_cost = self.get_path_cost(bid_from,
                                               self._point_to_pose(task_msg.location))

                print("path_cost={0}".format(path_cost))

                bid_msg = self._construct_bid_msg(task_msg.task.task_id,
                                                  self.robot_name,
                                                  path_cost)

                rospy.logdebug("bid_msg:\n{0}".format(pp.pformat(bid_msg)))
            
                stamp(bid_msg)
                self.bid_pub.publish(bid_msg)

        elif announce_msg.mechanism == 'SSI':
            rospy.loginfo("mechanism == SSI")
            
            # Get the cumulative cost of all tasks in out agenda so far
            (c_cost, bid_from) = self._cumulative_cost(self.current_pose,
                                                       self.agenda,
                                                       greedy=True)
            minimum_cost = None
            minimum_cost_task_id = None

            for task_msg in announce_msg.tasks:                
                #path_cost = self.get_path_cost(bid_from,
                #                               self._point_to_pose(task_msg.location))

                new_task_cost = self.get_path_cost(bid_from,
                                                   self._point_to_pose(task_msg.location))
                
                path_cost = c_cost + new_task_cost

                rospy.logdebug("path_cost={0}".format(path_cost))

                if minimum_cost is None or path_cost < minimum_cost:
                    minimum_cost = path_cost
                    minimum_cost_task_id = task_msg.task.task_id

            rospy.logdebug("minimum_cost={0} to task {1}".format(minimum_cost,
                                                                 minimum_cost_task_id))

            bid_msg = self._construct_bid_msg(minimum_cost_task_id,
                                              self.robot_name,
                                              minimum_cost)            

            rospy.logdebug("bid_msg:\n{0}".format(pp.pformat(bid_msg)))

            stamp(bid_msg)
            self.bid_pub.publish(bid_msg)

        else:
            rospy.logerr("bid(): mechanism '{0}' not supported".format(self.mechanism))
            
    def won(self, award_msg):
        rospy.loginfo("state: won")

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

        # Sanity check
        if not goal_task:
            self.fsm.no_tasks()

        self.fsm.goal_chosen(goal_task=goal_task)

    def on_amcl_pose_received(self, amcl_pose_msg):
        # amcl_pose_msg is typed as geometry_msgs/PoseWithCovarianceStamped.
        # We'll just keep track of amcl_pose_msg.pose.pose, which is typed as
        # geometry_msgs/Pose
        self.current_pose = amcl_pose_msg.pose.pose

    def on_experiment_event_received(self, event_msg):
        if event_msg.event == 'BEGIN_EXECUTION':
            self.ok_to_execute = True
        elif event_msg.event == 'END_EXPERIMENT':
            self.shutdown()

    def send_goal(self, e):
        rospy.loginfo("state: send_goal")

        goal_task = e.goal_task

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
        goal_msg.target_pose.pose.orientation.w = 1.0

        rospy.logdebug("Sending goal: {0}".format(goal_msg))
        self.aclient.send_goal(goal_msg)

        self.fsm.goal_sent(goal_task=goal_task)

    def moving(self, e):
        goal_task = e.goal_task

        # Wait until the goal is reached
        self.aclient.wait_for_result()

        self.fsm.goal_reached(goal_task=goal_task)

    def task_success(self, e):
        rospy.loginfo("state: task_success")

        goal_task = e.goal_task

        # Send a message to mark the end of this task's execution
        end_task_msg = multirobot_common.msg.TaskStatus()
        end_task_msg.robot_id = self.robot_name
        end_task_msg.task_id = goal_task.task_id
        end_task_msg.status = 'SUCCESS'
        stamp(end_task_msg)
        self.task_status_pub.publish(end_task_msg)

        goal_task.completed = True

        self.fsm.resume()

    def task_failure(self, e):
        rospy.loginfo("state: task_failure")

        goal_task = e.goal_task

        # Send a message to mark the end of this task's execution
        end_task_msg = multirobot_common.msg.TaskStatus()
        end_task_msg.robot_id = self.robot_name
        end_task_msg.task_id = goal_task.task_id
        end_task_msg.status = 'FAILURE'
        stamp(end_task_msg)
        self.task_status_pub.publish(end_task_msg)

        goal_task.completed = False

        self.fsm.resume()

    def idle(self, e):
        print("state: idle")

        # We idle here unless two conditions are true:
        # 1. We have tasks in our agenda
        # 2. self.ok_to_execute == True
        while not self.agenda or not self.ok_to_execute:
#            print("idle..")
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

    def shutdown(self):
        # Do any cleanup here before shutting down
        
        print("Shutting down...")
        sys.exit(0)

if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv=sys.argv[1:])
        print "arguments: {0}".format(argv)
        rc = RobotController(*argv)
    except rospy.ROSInterruptException:
        pass
