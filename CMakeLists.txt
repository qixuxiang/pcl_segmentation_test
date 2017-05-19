cmake_minimum_required( VERSION 2.8 )
project( segmentation_test )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# opencv 
find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen 
include_directories( "/usr/include/eigen3/" )

# pcl 
find_package( PCL REQUIRED COMPONENT io common visualization sample_consensus segmentation filters)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_executable( segTest main.cpp )
target_link_libraries( segTest ${OpenCV_LIBS} ${PCL_LIBRARIES} )
