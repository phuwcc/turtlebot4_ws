#!/usr/bin/env bash
# Prefer: python3 scripts/pc_config.py (asks count and validates every address).
# The setup script generates this file outside Git; do not source placeholders.
export TB4_ROBOT_COUNT=2
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Server 0 is the first entry, server 1 the second. No empty robot3 entry.
export ROS_DISCOVERY_SERVER='ROBOT1_PI_IP:11811;ROBOT2_PI_IP:11811;'
export ROS_SUPER_CLIENT=True
export ROS_LOCALHOST_ONLY=0
