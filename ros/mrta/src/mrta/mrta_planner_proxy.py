"""mrta_planner_proxy

This class is used by both mrta_robot_controllers and mrta_auctioneers for
the purpose of finding global plans and their costs.

"""

# Standard Python modules
import math
import time

# ROS stuff
import rospy
import dynamic_reconfigure.client
import geometry_msgs.msg
import nav_msgs.msg
import nav_msgs.srv
import std_srvs.srv

import pprint

pp = pprint.PrettyPrinter(indent=4)


class PlannerProxy:
    # def __init__(self, robot_name, plan_srv_name, cc_srv_name, glob_obs_layer_srv, glob_static_layer_srv):
    def __init__(self, robot_name):
        self.robot_name = robot_name                        # Name of the robot, possible a dummy (e.g., "robot_0")

        # self.plan_srv_name = plan_srv_name                  # Global planner service name
        # self.cc_src_name = cc_srv_name                      # Clear costmaps service name
        # self.glob_obs_layer_srv = glob_obs_layer_srv        # Global obstacle layer name
        # self.glob_static_layer_name = glob_static_layer_srv # Global static layer name

        # self.plan_srv_name = "/{0}/move_base_node/NavfnROS/make_plan".format(self.robot_name)
        self.plan_srv_name = "/{0}/move_base_node/GlobalPlanner/make_plan".format(self.robot_name)
        # self.plan_srv_name = "/{0}/move_base_node/HRTeamPlanner/make_plan".format(self.robot_name)
        self.cc_srv_name = "/{0}/move_base_node/clear_costmaps".format(self.robot_name)
        self.glob_obs_layer_srv = "/{0}/move_base_node/global_costmap/obstacle_layer".format(self.robot_name)
        self.glob_obs_layer_enabled = '/{0}/move_base_node/global_costmap/obstacle_layer/enabled'.format(self.robot_name)
        self.glob_static_layer_srv = '/{0}/move_base_node/global_costmap/static_layer'.format(self.robot_name)
        self.glob_static_layer_enabled = '/{0}/move_base_node/global_costmap/static_layer/enabled'.format(self.robot_name)

        # Start the ROS node
        # rospy.init_node("mrta_planner_proxy")

        # Define and initialize service clients
        self.obs_layer_param_client = None
        self.static_layer_param_client = None
        self._nav_plan_client = None
        self._clear_costmaps_client = None

        self.init_service_clients()

        # Maintain a cache of path distances between points so that we don't
        # have to look them up (via an expensive service call) every time.
        # Keys are tuples in the form of ((start_x,start_y),(goal_x,goal_y))#
        # and values are distances in meters (floats).
        self._path_distance_cache = {}

    def _disable_obstacle_layer(self):
        """ Disable the obstacle layer of the global costmap """

        rospy.loginfo("({0}) Disabling obstacle layer in global costmap".format(self.robot_name))
        self.obs_layer_param_client.update_configuration({'enabled': False})

        # Wait until we can confirm the layer is actually enabled
        while rospy.get_param(self.glob_obs_layer_enabled):
            self.rate.sleep()

        time.sleep(0.25)
        rospy.loginfo("({0}) ...disabled.".format(self.robot_name))

    def _enable_obstacle_layer(self):
        """ Enable the obstacle layer of the global costmap """

        rospy.loginfo("({0}) Enabling obstacle layer in global costmap".format(self.robot_name))
        self.obs_layer_param_client.update_configuration({'enabled': True})

        # Wait until we can confirm the layer is actually enabled
        while not rospy.get_param(self.glob_obs_layer_enabled):
            self.rate.sleep()

        time.sleep(0.25)
        rospy.loginfo("({0}) ...enabled.".format(self.robot_name))

    def _disable_static_layer(self):
        """ Disable the static layer of the global costmap """

        rospy.loginfo("({0}) Disabling static layer in global costmap".format(self.robot_name))
        self.static_layer_param_client.update_configuration({'enabled': False})

        # Wait until we can confirm the layer is actually enabled
        while rospy.get_param(self.glob_static_layer_enabled):
            self.rate.sleep()

        time.sleep(0.25)
        rospy.loginfo("({0}) ...disabled.".format(self.robot_name))

    def _enable_static_layer(self):
        """ Enable the static layer of the global costmap """

        rospy.loginfo("({0}) Enabling static layer in global costmap".format(self.robot_name))
        self.static_layer_param_client.update_configuration({'enabled': True})

        # Wait until we can confirm the layer is actually enabled
        while not rospy.get_param(self.glob_static_layer_enabled):
            self.rate.sleep()

        time.sleep(0.25)
        rospy.loginfo("({0}) ...enabled.".format(self.robot_name))

    def init_service_clients(self):
        rospy.loginfo("PlannerProxy: Starting obstacle_layer parameter client:")
        # rospy.loginfo(self.glob_obs_layer_srv)
        self.obs_layer_param_client = dynamic_reconfigure.client.Client(self.glob_obs_layer_srv, timeout=5)

        rospy.loginfo("PlannerProxy: Starting static_layer parameter client")
        # rospy.loginfo(self.glob_static_layer_srv)
        self.static_layer_param_client = dynamic_reconfigure.client.Client(self.glob_static_layer_srv, timeout=5)

        # Set up client for the service that makes path plans
        rospy.loginfo("PlannerProxy: Starting 'make_plan' service client")
        rospy.logdebug("PlannerProxy: Waiting for service {0}".format(self.plan_srv_name))
        rospy.wait_for_service(self.plan_srv_name)
        rospy.logdebug("PlannerProxy: Service {0} ready.".format(self.plan_srv_name))

        self._nav_plan_client = rospy.ServiceProxy(self.plan_srv_name,
                                                   nav_msgs.srv.GetPlan)

        # Set up client for the service that clear costmaps
        rospy.loginfo("PlannerProxy: Starting 'clear_costmaps' service client")

        rospy.logdebug("PlannerProxy: Waiting for service {0}".format(self.cc_srv_name))
        rospy.wait_for_service(self.cc_srv_name)
        rospy.logdebug("PlannerProxy: Service {0} ready.".format(self.cc_srv_name))

        self._clear_costmaps_client = rospy.ServiceProxy(self.cc_srv_name,
                                                         std_srvs.srv.Empty)

    def _is_reachable_distance(self, distance):
        """
        True if the distance is some 'reasonable' number (e.g. less than 10000).
        This test is used in several places. It's been moved into a function so
        that the actual test can be defined/updated in one place.

        :param distance: float
        :return: True/False
        """

        if distance < 10000:
            return True

        return False

    @staticmethod
    def _pose_to_posestamped(pose, frame_id='map'):
        """ Convert a geometry_msgs.msg.Pose to a geometry_msgs.msg.PoseStamped

        :param geometry_msgs.msg.Pose: point
        :return: A new g.m.PoseStamped object
        :rtype: geometry_msgs.msg.PoseStamped
        """
        pose_stamped_msg = geometry_msgs.msg.PoseStamped()

        pose_stamped_msg.header.stamp = rospy.get_rostime()
        pose_stamped_msg.header.frame_id = frame_id
        pose_stamped_msg.pose = pose

        return pose_stamped_msg

    @staticmethod
    def _point_to_pose(point):
        """ Convert a mrta.Point to a geometry_msgs.msg.Pose

        :param mrta.Point: point
        :return: A new g.m.Pose object
        :rtype: geometry_msgs.msg.Pose
        """
        pose_msg = geometry_msgs.msg.Pose()
        pose_msg.position = point
        pose_msg.orientation.w = 1.0

        return pose_msg

    def _make_nav_plan(self, start, goal):
        """ Create a global plan between start and goal

        :param geometry_msgs.msg.Pose: start
        :param geometry_msgs.msg.Pose: goal
        :return: A path from start to goal
        :rtype: geometry_msgs.msg.Pose[]
        """

        # We need to convert start and goal from type geometry_msgs.msg.Pose
        # to type geometry_msgs.PoseStamped. tolerance (from the docs):
        # "If the goal is obstructed, how many meters the planner can
        # relax the constraint in x and y before failing."
        start_stamped = self._pose_to_posestamped(start)
        goal_stamped = self._pose_to_posestamped(goal)

        req = nav_msgs.srv.GetPlanRequest()
        req.start = start_stamped
        req.goal = goal_stamped
        req.tolerance = 0.1

        try:
            rospy.logdebug("PlannerProxy: Waiting for service {0}".format(self.plan_srv_name))
            rospy.wait_for_service(self.plan_srv_name)
            rospy.logdebug("PlannerProxy: Service ready.")
            resp = self._nav_plan_client(req)

            rospy.logdebug("PlannerProxy: Got plan:\n{0}".format(pp.pformat(resp)))

            return resp.plan.poses

        except rospy.ServiceException, e:
            rospy.logerr("PlannerProxy: Service call failed: {0}".format(e))

    def _lookup_path_cost(self, start, goal):
        """
        Call the global planner service to obtain a path from start to goal
        and then calculate its cost. The cost is the sum of distances between
        points in the path PLUS the angular changes (in radians) needed to
        re-orient from every point to its successor.

        This is taken from the alufr ROS package. See getPlanCost() in
        navstack_module.cpp at:
        https://code.google.com/p/alufr-ros-pkg/source/browse/.

        :param geometry_msgs.msg.Pose: start
        :param geometry_msgs.msg.Pose: goal
        :return: The cost of the shortest path from start to goal
        :rtype: float
        """

        # rospy.loginfo("Getting path from {0}...".format(self.plan_srv_name))

        # 'path' is a list of objects of type geometry_msgs.PoseStamped
        path = self._make_nav_plan(start, goal)

        # rospy.loginfo('...got path')

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
                # yaw_to = tf.transformations.euler_from_quaternion(to_pose.orientation)[2]
                # yaw_from = tf.transformations.euler_from_quaternion(from_pose.orientation)[2]
                # yaw_delta = math.fabs(self._normalize_angle(yaw_to-yaw_from))
                # path_cost += yaw_delta

                from_pose = pose_stamped.pose

        # If a path can't be calculated, make its cost the maximum int value
        # This comparison should be safe in Python!
        if path_cost == 0.0:
            path_cost = float(sys.maxint)

        rospy.logdebug("PlannerProxy ({0}): Path cost from ({1},{2}) to ({3},{4}) is {5}".format(
            self.robot_name,
            start.position.x, start.position.y,
            goal.position.x, goal.position.y,
            path_cost))

        return path_cost

    def get_path_cost(self, start, goal):
        """
        Get the path distance from start to goal. Performs a service call to
        the global planner if we don't find the distance in a cache.

        :param geometry_msgs.msg.Pose start: start point
        :param geometry_msgs.msg.Pose goal: end point
        :return: distance in meters
        :rtype: float
        """

        # Let's consider the distances start->goal and goal->start
        # to be equivalent. We'll need keys to look up (and possibly
        # store) distances for each direction
        start_tuple = (start.position.x, start.position.y)
        goal_tuple = (goal.position.x, goal.position.y)
        sg_key = (start_tuple, goal_tuple)
        gs_key = (goal_tuple, start_tuple)

        # The distance to return
        distance = None

        # First check the cache
        if sg_key in self._path_distance_cache and self._is_reachable_distance(self._path_distance_cache[sg_key]):
            distance = self._path_distance_cache[sg_key]
            rospy.logdebug("  CACHE HIT (sg), distance=={0}".format(distance))
        elif gs_key in self._path_distance_cache and self._is_reachable_distance(self._path_distance_cache[gs_key]):
            distance = self._path_distance_cache[gs_key]
            rospy.logdebug("  CACHE HIT (gs), distance=={0}".format(distance))

        # Otherwise call the planner to find the distance
        else:
            rospy.logdebug("  CACHE MISS, LOOKING UP...")

            distance = self._lookup_path_cost(start, goal)

            # Store the distance in the cache
            self._path_distance_cache[sg_key] = distance
            self._path_distance_cache[gs_key] = distance

        rospy.logdebug("({0}): get_path_cost: ({1},{2})->({3},{4}) == {5}".format(
            self.robot_name,
            start.position.x, start.position.y,
            goal.position.x, goal.position.y,
            distance))

        return distance
