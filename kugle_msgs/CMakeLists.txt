cmake_minimum_required(VERSION 2.8.3)
project(kugle_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs actionlib_msgs)

# Inspired from https://github.com/ros/common_msgs/tree/jade-devel/nav_msgs

#add_message_files(
#  DIRECTORY msg
#  FILES
#  #messageFileHere.msg
#)

#add_service_files(
#  DIRECTORY srv
#  FILES
#  #serviceFileHere.srv
#)

#add_action_files(
#  FILES
#  #actionFileHere.action
#)

#generate_messages(DEPENDENCIES geometry_msgs std_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs actionlib_msgs)
