cmake_minimum_required(VERSION 2.8.3)
project(dmotion)
add_definitions(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS roscpp
               geometry_msgs
               message_generation
               dprocess)

# find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  ActionCmd.msg
  GaitType.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dmotion
  CATKIN_DEPENDS dprocess
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(dmotion
  src/dmotion.cpp include/dmotion/dmotion.hpp
  src/GaitStateManager.cpp include/dmotion/GaitStateManager.hpp)
add_dependencies(dmotion ${PROJECT_NAME}_generate_messages_cpp)

add_executable(dmotion_node src/main.cpp)
target_link_libraries(dmotion_node dmotion ${catkin_LIBRARIES})
add_dependencies(dmotion_node ${PROJECT_NAME}_generate_messages_cpp)

