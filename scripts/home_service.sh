#!/bin/sh

# Roscore
xterm  -e  " source devel/setup.bash; roscore" & 
sleep 4

# Turtlebot with hokuyo Gazebo
xterm  -e  " roslaunch demo myrobot_world.launch" &
sleep 2

# Keyboard Control
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 2

# pick_objects
xterm  -e  " roslaunch pick_objects pick_objects.launch" &
sleep 2

# Navigation
xterm  -e  " roslaunch demo navigation_demo.launch" &
sleep 2

# Turtlebot RVIZ
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 2

# Add markers
xterm  -e  " rosrun add_markers add_markers" &
sleep 2



