cmake_minimum_required(VERSION 2.8.3)
project(dvision)

add_definitions(-std=c++11 -Wall -g -Wextra -Wno-unused-result) #-Wfloat-equal -Weffc++ -pedantic

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

find_package(catkin REQUIRED
  COMPONENTS roscpp
  geometry_msgs
  message_generation
  dprocess
  dtransmit
  dconfig
  darknetcxx)

find_package(SuiteSparse REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)

#find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  SaveImg.msg
  Line.msg
  VisionInfo.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
    LIBRARIES dvisionlib ${OpenCV_LIBS}
  #  CATKIN_DEPENDS dprocess
  #  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR}) #${Boost_INCLUDE_DIRS})

#SET (G2O_LIBS csparse cxsparse g2o_csparse_extension g2o_cli g2o_core g2o_interface g2o_parser g2o_solver_cholmod
#g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp
#g2o_types_sba g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d)

SET(G2O_LIBS 
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_CLI_LIBRARY}
    ${G2O_SOLVER_CHOLMOD}
    ${G2O_SOLVER_CSPARSE}
    ${G2O_SOLVER_CSPARSE_EXTENSION}
    ${G2O_SOLVER_DENSE}
    ${G2O_SOLVER_PCG}
    ${G2O_SOLVER_SLAM2D_LINEAR}
    ${G2O_SOLVER_STRUCTURE_ONLY}
    ${G2O_SOLVER_EIGEN}
    ${G2O_TYPES_DATA}
    ${G2O_TYPES_ICP}
    ${G2O_TYPES_SBA}
    ${G2O_TYPES_SCLAM2D}
    ${G2O_TYPES_SIM3}
    ${G2O_TYPES_SLAM2D}
    ${G2O_TYPES_SLAM3D}
    )


add_library(dvisionlib
  src/dvision.cpp
  src/frame.cpp
  src/parameters.cpp
  src/camera/camera.cpp
  src/camera/camera_dummy.cpp
  src/camera/calibration.cpp
  src/camera/distortionModel.cpp
  src/camera/ipm.cpp
  src/camera/projection.cpp
  src/detector/ball_detector.cpp
  src/detector/circle_detector.cpp
  src/detector/field_detector.cpp
  src/detector/goal_detector.cpp
  src/detector/line_detector.cpp
  src/localization/localization.cpp
  src/tools/line_segment.cpp
  src/tools/utils.cpp
  src/tools/kalman.cpp
  )
target_link_libraries(dvisionlib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CSPARSE_LIBRARY} ${G2O_LIBS}) # ${Boost_LIBRARIES})
add_dependencies(dvisionlib ${PROJECT_NAME}_generate_messages_cpp)

add_executable(dvision_node src/main.cpp)
target_link_libraries(dvision_node dvisionlib ${catkin_LIBRARIES})
add_dependencies(dvision_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(calib src/camera/calibration.cpp)
target_link_libraries(calib ${OpenCV_LIBS})

add_executable(cvcalib src/camera/offical-calibration.cpp)
target_link_libraries(cvcalib ${OpenCV_LIBS})

add_executable(capture src/tools/capture.cpp)
target_link_libraries(capture dvisionlib)

## TEST
catkin_add_gtest(test_cameraDummy test/test_cameraDummy.cpp)
target_link_libraries(test_cameraDummy dvisionlib)
target_link_libraries(test_cameraDummy ${catkin_LIBRARIES})

catkin_add_gtest(test_distortionModel test/test_distortionModel.cpp)
target_link_libraries(test_distortionModel dvisionlib)
target_link_libraries(test_distortionModel ${catkin_LIBRARIES})

catkin_add_gtest(test_projection test/test_projection.cpp)
target_link_libraries(test_projection dvisionlib)
target_link_libraries(test_projection ${catkin_LIBRARIES})

catkin_add_gtest(test_localization test/test_localization.cpp)
target_link_libraries(test_localization dvisionlib)
target_link_libraries(test_localization ${catkin_LIBRARIES})
