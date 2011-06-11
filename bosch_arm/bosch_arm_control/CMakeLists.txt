cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
#rosbuild_add_executable(src/bosch_arm_teleop bosch_arm_teleop.cpp)
rosbuild_add_library(${PROJECT_NAME} src/daq.cpp)
rosbuild_add_library(${PROJECT_NAME} src/robot.cpp)
rosbuild_add_library(${PROJECT_NAME} src/s626mod.c)
rosbuild_add_library(${PROJECT_NAME} src/s626core.c)
rosbuild_add_library(${PROJECT_NAME} src/bosch_arm_hardware.cpp)
rosbuild_add_executable(bosch_arm_servo src/realtime_loop.cpp)
#rosbuild_add_executable(bosch_arm_node src/bosch_arm_driver.cpp)
#rosbuild_add_executable(service_test src/service_test.cpp)
#rosbuild_add_executable(bosch_arm_teleop src/teleop.cpp)
#rosbuild_add_executable(testsin src/testsin.cpp)
target_link_libraries(bosch_arm_servo ${PROJECT_NAME})
#common commands for building c++ executables and libraries

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
