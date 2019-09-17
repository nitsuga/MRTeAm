#!/bin/bash

STARTUP_HOME=/home/interact/GIT/mrta/startup

source /opt/ros/kinetic/setup.bash
export TURTLEBOT3_MODEL=burger

# MRTeAm workspace
source /home/interact/mrteam_ws/devel/setup.bash

TMUX_NAME=mrta_robot_controller
SCRIPT_NAME=mrta_robot_controller

start () {
echo "Sleeping for 10"
echo "Starting..."
echo "  - ${TMUX_NAME}"
echo "tmux new -d -s ${TMUX_NAME} ${STARTUP_HOME}/scripts/${SCRIPT_NAME}.sh"
tmux new-session -d -s ${TMUX_NAME} ${STARTUP_HOME}/scripts/${SCRIPT_NAME}.sh
RETVAL=$?
echo "Done"
return $RETVAL
}

stop(){
echo "Stopping..."
echo "  - ${TMUX_NAME}"
# "|| true" to ignore error exit code from tmux if the session is closed already.
tmux send-keys -t ${TMUX_NAME} Enter C-c Enter C-d || true
echo "      SIGTERM sent"
/bin/sleep 3s
tmux kill-session -t ${TMUX_NAME} || true
echo "      Killing session"
RETVAL=$?
echo "Done"
return $RETVAL
}

case "$1" in
    start)
        start
    ;;
    stop)
        stop
    ;;
    *)
        echo $"Usage: script {start|stop}"
        exit 3
    ;;
esac

exit $RETVAL
