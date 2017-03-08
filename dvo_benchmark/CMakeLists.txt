cmake_minimum_required(VERSION 2.4.6)

project(dvo_benchmark)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O3 -msse4 -msse4.2")

find_package(catkin REQUIRED COMPONENTS roscpp rospy dvo_core dvo_ros dvo_slam)

include_directories(include ${catkin_INCLUDE_DIRS})

# Eigen3 library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# VTK library
find_package(PCL REQUIRED COMPONENTS common visualization)
include_directories(${PCL_INCLUDE_DIRS})

catkin_package()

add_executable(dvo_benchmark
     src/benchmark_slam.cpp
)

#rosbuild_add_executable(experiment
#     src/experiment.cpp
#)

target_link_libraries(dvo_benchmark
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# create output directory - just convenience for our users
file(MAKE_DIRECTORY "${PROJECT_SOURCE_DIR}/output")
file(MAKE_DIRECTORY "${PROJECT_SOURCE_DIR}/output/video")
