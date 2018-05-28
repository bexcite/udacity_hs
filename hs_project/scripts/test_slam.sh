#!/bin/sh
# TODO: From Testing SLAM
x-terminal-emulator -e roslaunch hs_project turtlebot_world_mansion.launch
# x-terminal-emulator -e roslaunch turtlebot_gazebo turtlebot_world.launch gui:=false world_file:=../worlds/mansion_furnished.world &
sleep 5 &&
x-terminal-emulator -e roslaunch turtlebot_navigation gmapping_demo.launch &
sleep 5 &&
x-terminal-emulator -e roslaunch turtlebot_rviz_launchers view_navigation.launch &
sleep 5 &&
x-terminal-emulator -e roslaunch turtlebot_teleop keyboard_teleop.launch &
