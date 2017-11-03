cmake_minimum_required(VERSION 2.8.3)
project(trajectory_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(DCMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  sensor_msgs
  tf
  quadrotor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mavlink_message
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs nav_msgs sensor_msgs tf
#  DEPENDS system_lib
)

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(/usr/local/include/eigen3)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

INCLUDE_DIRECTORIES( include
    ${catkin_INCLUDE_DIRS}
)

add_executable(trajectory_node src/trajectory_node.cpp)
target_link_libraries(trajectory_node ${catkin_LIBRARIES})
