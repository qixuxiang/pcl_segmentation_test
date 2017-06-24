cmake_minimum_required(VERSION 2.8.3)
project(dbehavior)

find_package(catkin REQUIRED
  COMPONENTS rospy
             geometry_msgs
             message_generation)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BehaviorInfo.msg
  TeamInfo.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES dbehavior
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

