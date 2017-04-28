cmake_minimum_required(VERSION 2.8.3)
project(dvision)

add_definitions(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dvision
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
