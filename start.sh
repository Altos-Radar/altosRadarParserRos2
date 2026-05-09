#!/bin/bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
gnome-terminal --tab -t "rviz2" -- bash -c "rviz2 -d ./src/altosparser/altosradar.rviz; read"
sleep 2s
gnome-terminal --tab -t "altosParser" -- bash -c "
ros2 run altosparser altosparser --ros-args --params-file ./src/altosparser/param/altosParserParameters.yaml;
read"