#!/bin/sh
# Just some tests
# x-terminal-emulator  -e gazebo &
# sleep 5
x-terminal-emulator source /opt/ros/kinetic/setup.bash; roscore &
# sleep 5
# x-terminal-emulator  -e rosrun rviz rviz
