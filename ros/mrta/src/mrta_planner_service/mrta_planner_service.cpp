//
// Created by eric on 10/08/16.
//

#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>

#include <mrta_planner_service/mrta_planner_service.h>

namespace mrta_planner_service {

    bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp) {
        //make sure we have a costmap for our planner
        if (planner_costmap_ros_ == NULL) {
            ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
            return false;
        }
        tf::Stamped <tf::Pose> global_pose;
        if (!planner_costmap_ros_->getRobotPose(global_pose)) {
            ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
            return false;
        }
        geometry_msgs::PoseStamped start;
        //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
        if (req.start.header.frame_id == "")
            tf::poseStampedTFToMsg(global_pose, start);
        else
            start = req.start;

        //update the copy of the costmap the planner uses
        //clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);

        //first try to make a plan to the exact desired goal
        std::vector <geometry_msgs::PoseStamped> global_plan;
        if (!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()) {
            ROS_DEBUG_NAMED("move_base",
                            "Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
                            req.goal.pose.position.x, req.goal.pose.position.y);

            //search outwards for a feasible goal within the specified tolerance
            geometry_msgs::PoseStamped p;
            p = req.goal;
            bool found_legal = false;
            float resolution = planner_costmap_ros_->getCostmap()->getResolution();
            float search_increment = resolution * 3.0;
            if (req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
            for (float max_offset = search_increment;
                 max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
                for (float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
                    for (float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

                        //don't search again inside the current outer layer
                        if (x_offset < max_offset - 1e-9 && y_offset < max_offset - 1e-9) continue;

                        //search to both sides of the desired goal
                        for (float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

                            //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
                            if (y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

                            for (float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                                if (x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                                if (planner_->makePlan(start, p, global_plan)) {
                                    if (!global_plan.empty()) {

                                        //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                                        //(the reachable goal should have been added by the global planner)
                                        global_plan.push_back(req.goal);

                                        found_legal = true;
                                        ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)",
                                                        p.pose.position.x, p.pose.position.y);
                                        break;
                                    }
                                } else {
                                    ROS_DEBUG_NAMED("move_base", "Failed to find a plan to point (%.2f, %.2f)",
                                                    p.pose.position.x, p.pose.position.y);
                                }
                            }
                        }
                    }
                }
            }
        }

        //copy the plan into a message to send out
        resp.plan.poses.resize(global_plan.size());
        for (unsigned int i = 0; i < global_plan.size(); ++i) {
            resp.plan.poses[i] = global_plan[i];
        }

        return true;
    }

    int main(int argc, char **argv) {
        ros::init(argc, argv, "mrta_planner_service");

        ros::NodeHandle n;
        ros::NodeHandle private_nh("~");

        std::string global_planner;
        private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));

        //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
        planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
        planner_costmap_ros_->pause();

        //initialize the global planner
        try {
            planner_ = bgp_loader_.createInstance(global_planner);
            planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
        } catch (const pluginlib::PluginlibException& ex) {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
            exit(1);
        }

        ros::ServiceServer service = n.advertiseService("make_plan", planService);
        ROS_INFO("Ready to plan paths.");

        ros::spin();

        return 0;
    }

}