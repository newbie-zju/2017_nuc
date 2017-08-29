cmake_minimum_required(VERSION 2.8.3)
project(fisheye)

# using opencv 3.1 version:
set(OpenCV_DIR /usr/share/OpenCV)

#using opencv 2.4.8 version:
#set(OpenCV_DIR /usr/share/OpenCV)

find_package(OpenCV REQUIRED)
# include_directories(SYSTEM $(OpenCV_INCLUDE_DIRS) /usr/include)
# find_package("/usr/include/opencv2")
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
)
set(CMAKE_BUILD_TYPE "Release")
message(STATUS "opencv version: ${OpenCV_VERSION}")
message(STATUS "opencv lib: ${OpenCV_LIBS}")
message(STATUS "opencv include dir: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "opencv config path: ${OpenCV_CONFIG_PATH}")
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fisheye
#  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${fisheye_SOURCE_DIR}/include/fisheye
)
add_executable(${PROJECT_NAME}_node src/fisheye_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)
