#!/bin/bash
# This file is for docker container
export ROS_DOMAIN_ID=42
source /opt/ros/foxy/setup.bash
source /sim_ws/install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
