#!/bin/bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
gnome-terminal --tab -t "rviz2" -- bash -c "rviz2 -d ./src/altosparser/altosradar.rviz;read"
sleep 2s
gnome-terminal --tab -t "altosParserV4" -e 'bash -c "ros2 launch altosparser launchV4.py;read"'
sleep 2s
gnome-terminal --tab -t "altosParserRcu" -e 'bash -c "ros2 launch altosparser launchRcu.py;read"'