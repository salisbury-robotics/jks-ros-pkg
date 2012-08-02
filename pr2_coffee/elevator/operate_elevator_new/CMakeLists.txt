cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(operate_elevator_new)
SET(CMAKE_VERBOSE_MAKEFILE ON)

include_directories(~/lasik/stanford)

rospack_add_executable(bin/operate_elevator nodes/operate_elevator.cpp)

rospack_add_executable(bin/unwarp_images nodes/unwarp_images.cpp)

rospack_add_executable(bin/begin_operate_elevator nodes/begin_operate_elevator.cpp)
