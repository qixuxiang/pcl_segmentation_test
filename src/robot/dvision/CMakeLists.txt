cmake_minimum_required(VERSION 2.8.3)
project(dvision)
add_definitions(-std=c++11 -Wall -Wextra -Wfloat-equal -Wno-unused-result) #-Weffc++ -pedantic

find_package(catkin REQUIRED
  COMPONENTS roscpp
  geometry_msgs
  message_generation
  dprocess)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)


#find_package(Boost REQUIRED COMPONENTS system)

#add_message_files(
#  FILES
#)
#
#generate_messages(
#  DEPENDENCIES
#
#)

catkin_package(
  INCLUDE_DIRS include
  #  LIBRARIES dmotion
  #  CATKIN_DEPENDS dprocess
  #  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ) #${Boost_INCLUDE_DIRS})

add_library(dvisionlib
  src/dvision.cpp
  src/frame.cpp
  src/camera/camera.cpp
  src/camera/calibration.cpp
  src/camera/distortionModel.cpp
  # src/camera/projection.cpp
  )
target_link_libraries(dvisionlib ${catkin_LIBRARIES} ${OpenCV_LIBS}) # ${Boost_LIBRARIES})
#add_dependencies(dvisionlib ${PROJECT_NAME}_generate_messages_cpp)

add_executable(dvision_node src/main.cpp)
target_link_libraries(dvision_node dvisionlib ${catkin_LIBRARIES})
#add_dependencies(dmotion_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(calib src/camera/calibration.cpp)
target_link_libraries(calib ${OpenCV_LIBS})

add_executable(cvcalib src/camera/offical-calibration.cpp)
target_link_libraries(cvcalib ${OpenCV_LIBS})

add_executable(capture src/tools/capture.cpp)
target_link_libraries(capture dvisionlib)

## TEST
catkin_add_gtest(test_distortionModel test/test_distortionModel.cpp)
target_link_libraries(test_distortionModel dvisionlib)
target_link_libraries(test_distortionModel ${catkin_LIBRARIES})
