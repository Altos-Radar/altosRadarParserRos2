source devel/setup.bash
gnome-terminal --tab -t "rviz" -e  'bash -c "roslaunch rviz.launch;read"'
sleep 2s
rosparam load ./src/altosparser/param/altosParserParameters.yaml
gnome-terminal --tab -t "altosParser" -e  'bash -c "rosrun altosparser altosparser;read"'