cmake_minimum_required(VERSION 2.8.3)
project(mrta_dwa_local_planner)

find_package(catkin REQUIRED
        COMPONENTS
            base_local_planner
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            pcl_conversions
            roscpp
            tf
        )

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )
add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/MRTADWAPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES mrta_dwa_local_planner
    CATKIN_DEPENDS
        dynamic_reconfigure
        pluginlib
        roscpp
)

add_library(mrta_dwa_local_planner src/dwa_planner.cpp src/dwa_planner_ros.cpp)
target_link_libraries(mrta_dwa_local_planner ${catkin_LIBRARIES})
add_dependencies(mrta_dwa_local_planner dwa_local_planner_gencfg)
add_dependencies(mrta_dwa_local_planner nav_msgs_gencpp)

install(TARGETS mrta_dwa_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
