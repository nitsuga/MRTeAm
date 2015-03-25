cmake_minimum_required(VERSION 2.8.3)
project(hrteam_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  cmake_modules
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf
  visualization_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
  MakeNavPlan.srv
  SetCostmap.srv
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hrteam_planner
  CATKIN_DEPENDS base_local_planner cmake_modules costmap_2d dynamic_reconfigure geometry_msgs
  nav_core nav_msgs pluginlib roscpp tf visualization_msgs
)

###########
## Build ##
###########

## Declare a cpp library
add_library(hrteam_planner
  src/hrteam_planner.cpp
  src/astar.cpp
  src/Graph.cpp
  src/Map.cpp
  src/MapMarker.cpp
  src/MapVirtualWall.cpp
  src/MapWall.cpp
  src/PathPlanner.cpp)

target_link_libraries(hrteam_planner
  ${catkin_LIBRARIES}
  )

#############
## Install ##
#############

install(TARGETS hrteam_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   PATTERN ".gitignore" EXCLUDE
)

install(
  FILES
  bgp_plugin.xml
  map-11jan12.conf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
