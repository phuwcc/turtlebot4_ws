#!/usr/bin/env bash
# Copy this file outside Git and replace all three addresses with the Pi Wi-Fi IPs.
# Server 0 occupies the first list entry; server 1 the second; server 2 the third.
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER='ROBOT1_PI_IP:11811;ROBOT2_PI_IP:11811;ROBOT3_PI_IP:11811;'
export ROS_SUPER_CLIENT=True
export ROS_LOCALHOST_ONLY=0
