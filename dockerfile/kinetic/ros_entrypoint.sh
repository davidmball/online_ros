#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
roscore &
stdbuf -o L catkin build --no-status &&
(source "/home/root/catkin_ws/devel/setup.bash" &&
(roslaunch rosbridge_server rosbridge_websocket.launch &
rosrun tf2_web_republisher tf2_web_republisher &
until stdbuf -oL rosnode list | grep -m 1 "/rosbridge_websocket"; do sleep 0.2; done &&
(exec "$@")
))
