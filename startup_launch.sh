#!/usr/bin/env bash
# Place the kugle.service in /lib/systemd/system
# And enable the service on boot by running: 
#   sudo systemctl daemon-reload
#   sudo systemctl enable kugle.service
# Make sure this file has correct permissions: chmod u+x auto.sh

bash -c "source /opt/ros/kinetic/setup.bash && source /home/kugle/kugle_ws/devel/setup.bash && source /home/kugle/PrepareHostROS.sh && roslaunch kugle_bringup minimal.launch > /home/kugle/startup_launch.log &"

#source /opt/ros/kinetic/setup.bash 
#source /home/kugle/kugle_ws/devel/setup.bash

#export ROS_IP=
#export ROS_HOSTNAME=kugle.local
#export ROS_MASTER_URI=http://kugle.local:11311

#roslaunch kugle_bringup minimal.launch
