# Call this script by using "source ConnectROS.sh"
unset ROS_IP
unset ROS_HOSTNAME
unset ROS_MASTER_URI

machine_ip=(`hostname -I`)
export ROS_IP=${machine_ip[0]}
export ROS_MASTER_URI=http://kugle.local:11311
rostopic list
