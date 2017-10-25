cmake_minimum_required(VERSION 2.8.3)
project(pose_utils)

message(STATUS "cmake_prefix_path:" ${CMAKE_PREFIX_PATH})

set(CMAKE_VERBOSE_MAKEFILE "true")
include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_STD_CXX11)
if (NOT COMPILER_SUPPORTS_STD_CXX11)
  check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_STD_CXX0X)
endif ()

if (COMPILER_SUPPORTS_STD_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif (COMPILER_SUPPORTS_STD_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -march=native -Wno-unused-result")

find_package(catkin REQUIRED COMPONENTS
  #armadillo
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_utils
#  CATKIN_DEPENDS geometry_msgs nav_msgs
#  DEPENDS system_lib
)

find_package(Armadillo REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
    include
    )

add_library(pose_utils 
   ${ARMADILLO_LIBRARIES}
   src/pose_utils.cpp)
