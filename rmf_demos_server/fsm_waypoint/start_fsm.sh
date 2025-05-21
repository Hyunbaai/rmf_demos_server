#!/bin/bash
source /opt/ros/iron/setup.bash
source /home/zeta/ws_server/install/setup.bash

export CONFIG_FILE=/home/zeta/ws_server/install/rmf_demos/share/rmf_demos/config/gl/gl_deliveryRobot_config.yaml
export ROS_DOMAIN_ID=14

ros2 run fsm_waypoint fsm_waypoint_node
