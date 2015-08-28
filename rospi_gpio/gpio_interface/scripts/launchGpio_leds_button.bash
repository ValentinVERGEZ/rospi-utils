#!/bin/bash

## Launch the gpio_interface::leds_button_node with root user
# This is an ugly way to manage the caktin environments variables issue
# Warning : work with static paths

# Launch node
CATKIN_PATH="/home/pi/ros_catkin_ws"
source ${CATKIN_PATH}/devel/setup.bash
rosrun gpio_interface leds_button_node
