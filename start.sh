#!/bin/bash
source install/setup.bash
gnome-terminal --tab -t "rviz2" -e 'bash -c "rviz2 -d $(ros2 pkg prefix altosparser --share)/altosradar.rviz; read"'
sleep 2s
ros2 param load /altosrcu_node ./src/altosrcu/param/altosRcuParameters.yaml
gnome-terminal --tab -t "altosParser" -e 'bash -c "ros2 run altosparser altosparser;read"'