
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <hrteam_planner/Map.h>
#include <hrteam_planner/Graph.h>
#include <hrteam_planner/PathPlanner.h>

using std::string;

#ifndef HRTEAM_PLANNER_CPP
#define HRTEAM_PLANNER_CPP

namespace hrteam_planner {
  
  class HRTeamPlanner : public nav_core::BaseGlobalPlanner {
  public:
    
    HRTeamPlanner();  
    HRTeamPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    
    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start, 
		  const geometry_msgs::PoseStamped& goal, 
		  std::vector<geometry_msgs::PoseStamped>& plan
		  ); 

    bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

  private:
    /* Private members and functions here. */
    double node_spacing_;
    std::string tf_prefix_;

    costmap_2d::Costmap2DROS * costmap_ros_;
    costmap_2d::Costmap2D * costmap_;
    base_local_planner::WorldModel* world_model_; 

    ros::ServiceServer make_plan_srv_;

    bool initialized_;

    Graph * navgraph_;

    std::string hrteam_map_filename_;
    Map * read_map_file(std::string file_path);
    Map * hrteam_map_;

    PathPlanner * planner_;
  };
};

#endif
