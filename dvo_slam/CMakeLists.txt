cmake_minimum_required(VERSION 2.4.6)

project(dvo_slam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O3 -msse4 -msse4.2")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    dynamic_reconfigure
    dvo_core
    dvo_ros    
    std_msgs
    geometry_msgs
    message_generation
)

generate_dynamic_reconfigure_options(cfg/dvo_slam.cfg cfg/graph_visualizer.cfg)

add_message_files(
    FILES
    PoseStampedArray.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

include_directories(
    include
    cfg/cpp
    ${catkin_INCLUDE_DIRS}
    /usr/include/suitesparse
)

catkin_package(
    INCLUDE_DIRS include cfg/cpp
    LIBRARIES ${PROJECT_NAME}
)

# VTK library
find_package(PCL REQUIRED COMPONENTS common visualization)
include_directories(${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
  src/keyframe_tracker.cpp
  src/keyframe_graph.cpp
  src/keyframe_constraint_search.cpp
  src/camera_keyframe_tracking.cpp
  
  src/config.cpp
  
  src/tracking_result_evaluation.cpp
  src/local_map.cpp
  src/local_tracker.cpp
  
  src/constraints/constraint_proposal.cpp
  src/constraints/constraint_proposal_voter.cpp
  src/constraints/constraint_proposal_validator.cpp
  
  src/serialization/map_serializer_interface.cpp
  src/serialization/map_serializer.cpp
  
  src/visualization/graph_visualizer.cpp
  
  src/types.cpp
)

target_link_libraries(${PROJECT_NAME}
  g2o_core
  g2o_solver_dense
  g2o_solver_csparse
  g2o_csparse_extension
  cxsparse
  g2o_types_slam3d
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(camera_keyframe_tracker
  src/camera_keyframe_tracker_node.cpp
)

target_link_libraries(camera_keyframe_tracker
  ${PROJECT_NAME}
  glog
)
