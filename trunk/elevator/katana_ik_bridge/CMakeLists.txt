cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(katana_ik_bridge)
SET(CMAKE_VERBOSE_MAKEFILE ON)

#include_directories($ENV{ROS_ROOT}/../ros-pkg/drivers/robot/katana450/msg/cpp 
                     #$ENV{ROS_ROOT}/../ros-pkg/drivers/robot/katana450/srv/cpp)

rospack_add_executable(katana_ik_bridge katana_ik_bridge.cpp)
