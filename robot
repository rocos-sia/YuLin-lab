#!/bin/bash

sleep 10
source /home/sia/YuLin_lab/devel/setup.bash
roslaunch aruco_ros start.launch &
sleep 4
/home/sia/YuLin_lab/src/agvcom_linux01/agvcom_linux/agvcomm &
sleep 4
/home/sia/YuLin_lab/src/agvcom_linux02/agvcom_linux/agvcomm &


echo "It is a test"

# fi
