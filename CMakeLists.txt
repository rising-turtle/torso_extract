##
#  obtain realsense's frame, and compute human body orientation from the point cloud 
#

cmake_minimum_required(VERSION 2.8.3)

project(body_orientation)

set(CMAKE_CXX_FLAGS "$ENV{CXX_FLAGS} -O3 -w -g -ggdb -std=c++11")


## realsense libraries and includes 
include_directories(/usr/local/include/librealsense /usr/local/include/GLFW 
  /usr/local/include /usr/include/eigen3 /usr/include/python2.7)
link_directories(/usr/local/lib /usr/lib/x86_64-linux-gnu)

## glfw3: required for compiling show_pc or body_orientation 
find_package(glfw3 REQUIRED)

## opencv : required for test_histogram 
find_package(OpenCV REQUIRED)

add_subdirectory(src bin)
add_subdirectory(offline)
