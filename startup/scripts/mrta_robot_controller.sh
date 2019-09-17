#!/bin/bash

USER_HOME=/home/interact
source ${USER_HOME}/.bashrc
source ${USER_HOME}/mrteam_ws/devel/setup.bash

ROBOT_NAME=$(cat ${USER_HOME}/robotname)

echo "Starting mrta_robot controller for ${ROBOT_NAME}"

TURTLEBOT_NAME=${ROBOT_NAME} roslaunch mrta_robot_controller turtlebot3_no_ns_fkie.launch simulation:=false robot_name:=${ROBOT_NAME}

bash -i

