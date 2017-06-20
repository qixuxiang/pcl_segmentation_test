cmake_minimum_required(VERSION 2.8.3)
project(dmotion)
add_definitions(-std=c++11 -Wall -O2)

find_package(catkin REQUIRED
    COMPONENTS roscpp
               geometry_msgs
               message_generation
               dtransmit
               dconfig
               dprocess)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 3 REQUIRED)

add_message_files(
  FILES
  ActionCmd.msg
  MotionInfo.msg
)

add_service_files(
  FILES
  GetDelta.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES dmotion
#  CATKIN_DEPENDS dprocess
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(dmotionlib
  src/dmotion.cpp
  src/transitHub.cpp
  src/RobotStatus.cpp

  src/GaitStateManager.cpp
  include/dmotion/GaitStateManager.hpp

  src/GaitStateLib/GaitStateBase.cpp
  src/GaitStateLib/GaitStateWenxi.cpp
  src/GaitStateLib/GaitStateCrouch.cpp
  src/GaitStateLib/GaitStateStandup.cpp
  src/GaitStateLib/GaitStateKick.cpp
  src/GaitStateLib/GaitStateGoalie.cpp
  src/GaitStateLib/GaitStateSetupFrontDown.cpp
  src/GaitStateLib/GaitStateSetupBackDown.cpp

  src/GaitStateSupportLib/HumanRobot.cpp
  src/GaitStateSupportLib/HumanRobotInterface.cpp

  src/VecPos.cpp
  src/MotionData.cpp
  src/One_D_Filter.cpp
  src/GaitStateSupportLib/comEstimator.cpp
  src/GaitStateSupportLib/SplineThree.cpp
  src/GaitStateSupportLib/kalman_filter.cpp
  src/GaitStateSupportLib/onedCtrl.cpp
  )
target_link_libraries(dmotionlib ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(dmotionlib ${PROJECT_NAME}_generate_messages_cpp)

add_executable(dmotion_node src/main.cpp)
target_link_libraries(dmotion_node dmotionlib ${catkin_LIBRARIES})
add_dependencies(dmotion_node ${PROJECT_NAME}_generate_messages_cpp)
