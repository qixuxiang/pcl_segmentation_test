cmake_minimum_required(VERSION 2.8.3)
project(dvisualize)
#LIST(APPEND CMAKE_MODULE_PATH ${dvision_SOURCE_DIR}/cmake_modules)

add_definitions(-std=c++11 -Wall -Wextra -Wfloat-equal -Wno-unused-result) #-Weffc++ -pedantic

find_package(catkin REQUIRED
  COMPONENTS roscpp
  dvision
)

catkin_package(
#  INCLUDE_DIRS include
  #  LIBRARIES dmotion
  #  CATKIN_DEPENDS dprocess
  #  DEPENDS system_lib
)

find_package(Qt5 COMPONENTS Core Gui Quick Qml REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

include_directories(include ${catkin_INCLUDE_DIRS}) 
add_executable(dvisualize
    src/main.cpp
    src/qml.qrc
    )


target_link_libraries(dvisualize Qt5::Core Qt5::Gui Qt5::Qml Qt5::Quick)
