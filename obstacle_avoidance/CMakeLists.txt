cmake_minimum_required(VERSION 2.8.3)
project(obstacle_avoidance)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
SET(CMAKE_BUILE_TYPE RELEASE)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  dji_sdk
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   velocityPosition.msg
#   Message2.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES obstacle_avoidance
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${obstacle_avoidance_SOURCE_DIR}/include/obstacle_avoidance
#  ${irobot_detect_SOURCE_DIR}/include/irobot_detect
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/obstacle_avoidance.cpp
# )


add_executable(${PROJECT_NAME}_node src/obstacle_avoidance_node.cpp src/obstacleAvoidance.cpp src/LIDAR.cpp)

add_dependencies(obstacle_avoidance_node obstacle_avoidance_generate_messages_cpp)
add_dependencies(obstacle_avoidance_node dji_sdk_generate_messages_cpp)
add_dependencies(obstacle_avoidance_node iarc_mission_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
