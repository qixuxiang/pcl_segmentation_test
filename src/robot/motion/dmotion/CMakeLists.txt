cmake_minimum_required(VERSION 2.8.3)
project(dmotion)
add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS roscpp dprocess)

# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dmotion
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
message(${catkin_INCLUDE_DIRS})

add_executable(dmotion_node
  src/main.cpp
  src/dmotion.cpp include/dmotion/dmotion.hpp
  )

target_link_libraries(dmotion_node ${catkin_LIBRARIES})
