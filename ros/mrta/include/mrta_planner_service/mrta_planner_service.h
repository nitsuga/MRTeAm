//
// Created by eric on 10/08/16.
//
#ifndef MRTA_MRTA_PLANNER_SERVICE_H
#define MRTA_MRTA_PLANNER_SERVICE_H

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <nav_core/base_global_planner.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

namespace mrta_planner_service
{

    costmap_2d::Costmap2DROS *planner_costmap_ros_;

    boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
}

#endif //MRTA_MRTA_PLANNER_SERVICE_H
