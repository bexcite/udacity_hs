#!/bin/sh

# Test Pick Objects

x-terminal-emulator -e roslaunch hs_project turtlebot_world_mansion.launch &
sleep 5

x-terminal-emulator -e roslaunch hs_project amcl_demo_mansion.launch &
sleep 5

x-terminal-emulator -e roslaunch turtlebot_rviz_launchers view_navigation.launch &
sleep 5

x-terminal-emulator -e rosrun pick_objects pick_objects &
