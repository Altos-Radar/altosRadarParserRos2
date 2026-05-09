# altosRadarRos2Parser

## Build
1. git clone https://github.com/Altos-Radar/altosRadarParserRos2.git
2. cd altosRadarParserRos2
3. colcon build

## Set Parameters
altosRadarParserRos2/src/altosparser/param/altosParserParameters.yaml  
change numRadar to 1 for V4, 4 for RCU  
change topic name and installation parameters  

## Run
1. cd altosRadarParserRos2
2. bash start.sh