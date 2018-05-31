#!/bin/sh

# Add Markers test

x-terminal-emulator -e roslaunch hs_project turtlebot_world_mansion.launch
sleep 5
x-terminal-emulator -e roslaunch hs_project amcl_demo_mansion.launch &
sleep 5
# x-terminal-emulator -e roslaunch turtlebot_rviz_launchers view_navigation.launch &
x-terminal-emulator -e roslaunch hs_project view_navigation_marker.launch &
sleep 5
x-terminal-emulator -e rosrun add_markers add_markers &
