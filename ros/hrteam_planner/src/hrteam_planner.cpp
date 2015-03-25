#include <hrteam_planner/hrteam_planner.h>
#include <pluginlib/class_list_macros.h>

#include <iostream>
#include <fstream>

#include <hrteam_planner/Node.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(hrteam_planner::HRTeamPlanner, nav_core::BaseGlobalPlanner)

namespace hrteam_planner {

  HRTeamPlanner::HRTeamPlanner()
    : costmap_ros_(NULL), navgraph_(NULL), hrteam_map_(NULL), planner_(NULL) {}


  HRTeamPlanner::HRTeamPlanner(std::string name, costmap_2d::Costmap2DROS * costmap_ros)
    : costmap_ros_(NULL), navgraph_(NULL), hrteam_map_(NULL), planner_(NULL), initialized_(false) {
    initialize(name, costmap_ros);
  }

  void HRTeamPlanner::initialize(std::string name, costmap_2d::Costmap2DROS * costmap_ros) {

    if (!initialized_) {
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      /* ... */
      ros::NodeHandle private_nh("~/" + name);
      private_nh.param("node_spacing", node_spacing_, 23.0);
      const std::string default_map("/home/eric/ros_catkin_ws_hydro/install/share/hrteam_planner/map-11jan12.conf");
      private_nh.param("hrteam_map_filename", hrteam_map_filename_, default_map);
      cerr << "hrteam_map_filename_: " << hrteam_map_filename_ << std::endl;


      world_model_ = new base_local_planner::CostmapModel(*costmap_);

      ros::NodeHandle prefix_nh;
      tf_prefix_ = tf::getPrefixParam(prefix_nh);

      make_plan_srv_ = private_nh.advertiseService("make_plan", &HRTeamPlanner::makePlanService, this);
      hrteam_map_ = read_map_file(hrteam_map_filename_);

      navgraph_ = new Graph(hrteam_map_, node_spacing_);

      
      Node n;
      planner_ = new PathPlanner(navgraph_, *hrteam_map_, n, n);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }


  bool HRTeamPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
			       const geometry_msgs::PoseStamped& goal, 
			       std::vector<geometry_msgs::PoseStamped>& plan
			       ) {

    if (!initialized_) {
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();

    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
		costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;

    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start,start_tf);

    double useless_pitch, useless_roll, goal_yaw, start_yaw;
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

    double goal_x = goal.pose.position.x * 100.;
    double goal_y = goal.pose.position.y * 100.;
    double start_x = start.pose.position.x * 100.;
    double start_y = start.pose.position.y * 100.;

    /*
    Node source(1, start_x, start_y);
    Node target(1, goal_x, goal_y);

    planner_->setSource(source);
    planner_->setTarget(target);

    planner_->calcPath();
    */

    std::list<std::pair<int,int> > path = planner_->getPathXYBetween(start_x, start_y,
								     goal_x, goal_y);
    
    ros::Time plan_time = ros::Time::now();
    //for (int i = path.size() -1; i >= 0; i--) {
    for (std::list<std::pair<int,int> >::iterator it = path.begin();
	 it != path.end(); it++) {
      std::pair<int,int> cur_position = *it;

      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = goal.header.frame_id;
      pose.pose.position.x = (double)cur_position.first / 100.;
      pose.pose.position.y = (double)cur_position.second / 100.;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }

    /* ... */

  }

  bool HRTeamPlanner::makePlanService(nav_msgs::GetPlan::Request& req,
				      nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = costmap_ros_->getGlobalFrameID();

    return true;
  } 

  hrteam_planner::Map * HRTeamPlanner::read_map_file(std::string map_file_path) {
    std::string  cmd, label, tmp;
    int x1, y1, x2, y2, lx, ly, rx, ry; 
    bool first = true ;

    // default buffer==30
    int bufferSize = 30;
    
    hrteam_planner::Map *map = NULL;

    std::ifstream map_file( map_file_path.c_str() );

    if ( ! map_file.is_open() ) {
      std::cerr << "Can't read map file: " << map_file_path << std::endl;
      return map;
    }

    while( ! map_file.eof() ) {
      cmd = ""; label = ""; x1 = 0 ; y1 = 0 ; x2 = 0 ; y2 = 0 ; 
      map_file >> cmd ; 
      
      if (! ( cmd[0] == '/' && cmd[1] == '/' ) ) {
	// if the first command includes size, set the window for the specified values
	// else create the default map
	if ( first ){
	  first = false;
	  if ( cmd == "size" ) {
	    map_file >> x1 >> y1 >> bufferSize; 
	    map = new Map(x1, y1, bufferSize);
	    continue;
	  }
	}
	
	// process the command
	if ( cmd == "marker" ) { 
	  map_file >> label >> x1 >> y1 >> lx >> ly >> rx >> ry ;
	  map->addMarker(MapMarker(label, x1, y1, lx, ly, rx, ry));	
	}
	else if ( cmd == "wall" ) {
	  map_file >> label >> x1 >> y1 >> x2 >> y2 ; 
	  cout << "wall: " << x1 << y1 << x2 << y2 << endl;
	  map->addWall(label, x1, y1, x2, y2); 
	}
	else if ( cmd == "virtualWall" ) {	  
	  map_file >> label >> x1 >> y1 >> x2 >> y2 ; 
	  cout << "virtualwall: " << x1 << y1 << x2 << y2 << endl;
	  map->addVirtualWall(label, x1, y1, x2, y2); 
	}
	else {
	  cout << "Unknown map config command: '" << cmd << "'" << endl;
	  getline(map_file, tmp); 
	}
      }
      else {
	// ignore the rest of the line.
	getline(map_file, tmp); 
      }
    }

    return map;

  } // end read_map_file

}
