#!/bin/bash

ROBOT_NAME='Robot_A'
echo 'export robot name: '$ROBOT_NAME
export $ROBOT_NAME 
sudo chmod a+rw /dev/ttyUSB0
sudo chmod 777 /dev/ttyACM0

roslaunch anpl_active_demo launch_robot_sensors.launch robot_name:=$ROBOT_NAME is_lidar_usb:=True
