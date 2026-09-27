#!/bin/bash
source /opt/ros/foxy/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wlan0" priority="default" multicast="default" /><?Interfaces></General></-Domain></CycloneDDS>'
python3 /home/$USER/robot_ws/robot_core/server.py
