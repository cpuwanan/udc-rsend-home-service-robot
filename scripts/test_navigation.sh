#!/bin/sh

# Roscore
#xterm  -e  " source devel/setup.bash; roscore" & 
#sleep 2

# Turtlebot with hokuyo Gazebo
xterm  -e  " roslaunch demo myrobot_world.launch" &
sleep 1

# Keyboard Control
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 1

# Navigation
xterm  -e  " roslaunch demo navigation_demo.launch" &
sleep 1

# Turtlebot RVIZ
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 1
