#!/bin/bash
source /opt/ros/foxy/setup.bash
source /home/unitree/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wlan0" priority="default" multicast="default" /><?Interfaces></General></-Domain></CycloneDDS>'
export ROS_DOMAIN_ID=1
source /home/unitree/ros_ws/install/setup.bash
ros2 run camera_publisher video_publisher
