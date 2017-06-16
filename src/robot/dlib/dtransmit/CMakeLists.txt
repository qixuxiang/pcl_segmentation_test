cmake_minimum_required(VERSION 2.8.3)
project(dtransmit)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED roscpp)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dtransmit_lib
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

 add_library(dtransmit
   src/dtransmit.cpp
 )
target_link_libraries(dtransmit ${catkin_LIBRARIES} ${Boost_LIBRARIES})


# Testing
catkin_add_gtest(test_dtransmit test/test_dtransmit.cpp)
target_link_libraries(test_dtransmit dtransmit)
