cmake_minimum_required(VERSION 2.8.3)

if(POLICY CMP0046)
    cmake_policy(SET CMP0046 NEW)
endif()
if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

project(darknetcxx)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Add options
option(DARKNET_TEST "Build all tests." OFF) # Makes boolean 'test' available.
option(DARKNET_OPENCV "Using OpenCV" OFF)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
option(DARKNET_GPU "Using GPU" OFF)
option(DARKNET_GPROF "Using GPROF" OFF)
option(DARKNET_NO_DISPLAY, "No display" ON)
option(DARKNET_MALLOCATED, "No m_allocated" OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES darknetcxx
#  CATKIN_DEPENDS dprocess roscpp
#  DEPENDS system_lib
)

################
# flag for TX1 #
################

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    message("Processor: ${CMAKE_SYSTEM_PROCESSOR} add definition -DDARKNET_TX1")
    set(DARKNET_TX1 TRUE)
endif ()

########################
# find CUDA if DGPU=ON #
########################

if (DARKNET_GPU)
find_package(CUDA REQUIRED)
message("CUDA Version: ${CUDA_VERSION}")

SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3;-std=c++11)

if(DARKNET_TX1)
  SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_53,code=sm_53)
  add_definitions(-DDARKNET_TX1)
else(DARKNET_TX1)
  SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_61,code=sm_61)
endif(DARKNET_TX1)
endif(DARKNET_GPU)

#############################
# find OpemCV if DOPENCV=ON #
#############################

find_package(Threads REQUIRED)
if (DARKNET_OPENCV)
  find_package(OpenCV 3 REQUIRED COMPONENTS core imgproc highgui)
  # find_package(Qt5Core REQUIRED )
  # find_package(Qt5Widgets REQUIRED )
  # find_package(Qt5Gui REQUIRED )
  # find_package(Qt5Test REQUIRED )
  # find_package(Qt5Concurrent REQUIRED )
  # find_package(Qt5OpenGL REQUIRED )
  add_definitions(-DDARKNET_OPENCV)
  # include_directories(${OpenCV_INCLUDE_DIRS})
endif(DARKNET_OPENCV)

#############
# for flags #
#############

if (DARKNET_MALLOCATED)
  add_definitions(-DDARKNET_MALLOCATED)
  # message("ALLOCATED!")
endif(DARKNET_MALLOCATED)

if (DARKNET_GPU)
    add_definitions(-DDARKNET_GPU)
endif(DARKNET_GPU)

if (DARKNET_GPROF)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
endif(DARKNET_GPROF)

if (DARKNET_NO_DISPLAY)
    add_definitions(-DDARKNET_NO_DISPLAY)
endif(DARKNET_NO_DISPLAY)

# include_directories(./)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#######################
# lib darknetcxx-cuda #
#######################

if (DARKNET_GPU)
cuda_add_library(darknetcxx-cuda STATIC
        src/utils/cuda_utils.cu
        src/blas/blas.cu
        # Layers
        src/layer/activation/activations.cu
        src/layer/convolutional/convolutional.cu
        src/layer/maxpool/maxpool.cu
        )

target_include_directories(darknetcxx-cuda PUBLIC ${CUDA_INCLUDE_DIRS})
target_link_libraries(darknetcxx-cuda "-L/usr/local/cuda/lib64 -lcuda -lcudart -lcublas -lcurand -lcudnn")
endif(DARKNET_GPU)

##################
# lib darknetcxx #
##################

add_library(darknetcxx STATIC
        # darknet.cpp
        src/types/image.cpp
        src/types/data.cpp
        src/types/section.cpp
        src/network/network.cpp
        src/network/detector.cpp
        src/utils/utils.cpp
        src/utils/parser.cpp
        src/blas/blas.cpp
        src/blas/gemm.cpp
        src/blas/im2col.cpp
        # Layers
        src/layer/layer.cpp
        src/layer/activation/activations.cpp
        # layer/activation/activationlayer.cpp
        src/layer/convolutional/convolutional.cpp
        src/layer/maxpool/maxpool.cpp
        src/layer/region/region.cpp

        src/layer/region/box.cpp
        src/layer/connected/connected.cpp
        src/layer/dropout/dropout.cpp
        src/layer/detection/detection.cpp
        src/layer/local/local.cpp
        )

if (DARKNET_OPENCV)
    target_link_libraries(darknetcxx ${OpenCV_LIBS})
    target_include_directories(darknetcxx PUBLIC ${OpenCV_INCLUDE_DIRS})
endif(DARKNET_OPENCV)

set_target_properties(darknetcxx PROPERTIES
    COMPILE_FLAGS "-std=c++14 -Wall  -Wshadow -Wnon-virtual-dtor -pedantic")

if (DARKNET_GPU)
target_link_libraries(darknetcxx darknetcxx-cuda)
target_include_directories(darknetcxx PUBLIC ${CUDA_INCLUDE_DIRS})
endif(DARKNET_GPU)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_darknetcxx.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
