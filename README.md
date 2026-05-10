# altosRadarRos2Parser

## Build
1. git clone https://github.com/Altos-Radar/altosRadarParserRos2.git
2. cd altosRadarParserRos2
3. colcon build --symlink-install

## Set Parameters
V4: altosRadarParser/src/altosparser/param/altosParserV4.yaml  
RCU: altosRadarParser/src/altosparser/param/altosParserRCU.yaml  
set topic name, installation parameters, IP, port, etc.  

## Run
V4: bash startV4.sh  
RCU: bash startRcu.sh  
V4+RCU: bash startBoth.sh