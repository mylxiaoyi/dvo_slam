cmake_minimum_required(VERSION 2.4.6)
project(dvo_core)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O3 -msse4 -msse4.2")

find_package(catkin REQUIRED roscpp rospy)

include_directories(include)

# Eigen3 library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# VTK library
find_package(PCL REQUIRED COMPONENTS common visualization)
include_directories(${PCL_INCLUDE_DIRS})

#set(CMAKE_CXX_FLAGS
#   " -O3 -march=native"
#)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} dvo_visualization
    DEPENDS tbb boost_thread ${PCL_LIBRARIES}
)

add_library(${PROJECT_NAME} 
    src/core/interpolation.cpp
    src/core/intrinsic_matrix.cpp
    src/core/least_squares.cpp
    src/core/math_sse.cpp
    src/core/rgbd_image.cpp
    src/core/rgbd_image_sse.cpp
    src/core/point_selection.cpp
    src/core/surface_pyramid.cpp
    src/core/weight_calculation.cpp
    
    src/util/histogram.cpp
    
    src/visualization/camera_trajectory_visualizer.cpp
    
    src/dense_tracking.cpp
    src/dense_tracking_impl.cpp
    src/dense_tracking_config.cpp
)

target_link_libraries(${PROJECT_NAME} 
    tbb
    boost_thread
    ${PCL_LIBRARIES}
)

add_library(dvo_visualization
    src/visualization/async_point_cloud_builder.cpp
    src/visualization/point_cloud_aggregator.cpp
    src/visualization/pcl_camera_trajetory_visualizer.cpp
)

target_link_libraries(dvo_visualization
  ${PROJECT_NAME} 
)
