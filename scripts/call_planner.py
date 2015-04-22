#!/usr/bin/env python

import argparse
import geometry_msgs.msg
import math
import nav_msgs.msg
import nav_msgs.srv
import navfn.srv
import pprint
import rospy
import std_srvs.srv
import sys
import time

# Some constants. TODO: make these configurable.
#PLAN_SRV_NAME = '/robot_1/move_base_node/NavfnROS/make_plan'
PLAN_SRV_NAME = '/robot_1/move_base_node/GlobalPlanner/make_plan'
#PLAN_SRV_NAME = '/robot_1/move_base_node/HRTeamPlanner/make_plan'

CC_SRV_NAME = '/robot_1/move_base_node/clear_costmaps'

pp = pprint.PrettyPrinter(indent=2)

def _clear_costmaps():

    rospy.logdebug("Waiting for service {0}".format(CC_SRV_NAME))
    rospy.wait_for_service(CC_SRV_NAME)
    rospy.logdebug("Service ready.")

    try:
        clear_costmaps_proxy = rospy.ServiceProxy(CC_SRV_NAME,
                                                  std_srvs.srv.Empty)

        clear_costmaps_proxy()
        
        rospy.logdebug("cleared costmaps")

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {0}".format(e))

def _point_to_pose(point):
    pose_msg = geometry_msgs.msg.Pose()
    pose_msg.position = point
    pose_msg.orientation.w = 1.0

    return pose_msg

def _pose_to_posestamped(pose, frame_id='map'):
    pose_stamped_msg = geometry_msgs.msg.PoseStamped()
    
    pose_stamped_msg.header.stamp = rospy.get_rostime()
    pose_stamped_msg.header.frame_id = frame_id
    pose_stamped_msg.pose = pose
    
    return pose_stamped_msg

def init_node():
    rospy.init_node('call_planner')

def get_path_cost(plan):
    """
    Return the cost of a path from start to goal. The path is obtained
    from the navfn/MakeNavPlan service. The cost is the sum of distances
    between points in the path PLUS the angular changes (in randians)
    needed to re-orient from every point to its successor.
    
    This is taken from the alufr ROS package. See getPlanCost() in
    navstack_module.cpp at:
    https://code.google.com/p/alufr-ros-pkg/source/browse/.
    """
    
    path_cost = 0.0
    from_pose = None
    for pose_stamped in plan:
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


def call_planner(start_x=0.5, start_y=0.5, goal_x=5.015, goal_y=4.485):

    start_pt = geometry_msgs.msg.Point()
    start_pt.x = float(start_x)
    start_pt.y = float(start_y)
    #print("start_pt: {0}".format(pp.pformat(start_pt)))

    start_pose = _point_to_pose(start_pt)
    #print("start_pose: {0}".format(pp.pformat(start_pose)))

    goal_pt = geometry_msgs.msg.Point()
    goal_pt.x = float(goal_x)
    goal_pt.y = float(goal_y)
    goal_pose = _point_to_pose(goal_pt)

    #rospy.loginfo("Waiting for service {0}".format(PLAN_SRV_NAME))
    rospy.wait_for_service(PLAN_SRV_NAME)
    #rospy.loginfo("Service ready.")

    try:
        make_nav_plan = rospy.ServiceProxy(PLAN_SRV_NAME,
                                           nav_msgs.srv.GetPlan)
        
        # We need to convert start and goal from type geometry_msgs.Pose
        # to type geometry_msgs.PoseStamped. tolerance (from the docs):
        # "If the goal is obstructed, how many meters the planner can
        # relax the constraint in x and y before failing."
        start_stamped = _pose_to_posestamped(start_pose)
        goal_stamped = _pose_to_posestamped(goal_pose)
        
        req = nav_msgs.srv.GetPlanRequest()
        req.start = start_stamped
        req.goal = goal_stamped
        req.tolerance = 0.1

        #print("req: {0}".format(pp.pformat(req)))
        
        print("Clearing costmaps...")
        _clear_costmaps()
#        time.sleep(0.1)

        resp = make_nav_plan(req)
        
        #print("Got plan:\n{0}".format(pp.pformat(resp)))
        
        return resp.plan.poses
        
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {0}".format(e))


if __name__ == '__main__':

    init_node()

    parser = argparse.ArgumentParser(description="Call the global planner ({0}).".format(PLAN_SRV_NAME))

    parser.add_argument('start_x')
    parser.add_argument('start_y')
    parser.add_argument('goal_x')
    parser.add_argument('goal_y')
    
    args = parser.parse_args()

    print args.start_x

    n = 100

    for i in range(1,n+1):
        plan = call_planner(args.start_x, args.start_y, args.goal_x, args.goal_y)
        #pp.pprint(plan)
        cost = get_path_cost(plan)

        print("[call {0:02d}] cost from ({1}, {2}) to ({3}, {4}) is: {5}".format(i, 
                                                                                 args.start_x,
                                                                                 args.start_y,
                                                                                 args.goal_x,
                                                                                 args.goal_y,
                                                                                 cost))
        #time.sleep(0.5)
