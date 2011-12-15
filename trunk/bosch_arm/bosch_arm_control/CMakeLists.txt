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
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
#rosbuild_add_executable(src/bosch_arm_teleop bosch_arm_teleop.cpp)
rosbuild_add_library(${PROJECT_NAME} src/daq.cpp)
rosbuild_add_library(${PROJECT_NAME} src/s626mod.c)
rosbuild_add_library(${PROJECT_NAME} src/s626core.c)
rosbuild_add_library(${PROJECT_NAME} src/bosch_arm.cpp)
#rosbuild_add_library(${PROJECT_NAME} src/simple_cartesian_controller.cpp)
#rosbuild_add_library(${PROJECT_NAME} src/simple_joint_controller.cpp)
#rosbuild_add_library(${PROJECT_NAME} src/trajectory_controller3.cpp)
rosbuild_add_executable(old_node src/controller.cpp)
target_link_libraries(old_node ${PROJECT_NAME})

# #tuning the motors
# rosbuild_add_library(${PROJECT_NAME} src/lti.cpp)
# rosbuild_add_library(${PROJECT_NAME} src/motor_controller2.cpp)
# rosbuild_add_executable(bosch_arm_node src/main_motor2.cpp)

#############################################################
#track the surface with basket
rosbuild_add_library(${PROJECT_NAME} src/cartesian_controller2.cpp)
rosbuild_add_library(${PROJECT_NAME} src/action_cart.cpp)
rosbuild_add_executable(bosch_arm_node src/main_cart.cpp)
target_link_libraries(bosch_arm_node ${PROJECT_NAME})
##############################################################

#############################################################
#track the surface with long tip
#rosbuild_add_library(${PROJECT_NAME} src/cartesian_controller_longtip.cpp)
#rosbuild_add_library(${PROJECT_NAME} src/action_cart.cpp)
#rosbuild_add_executable(bosch_arm_node src/main_cart.cpp)
target_link_libraries(bosch_arm_node ${PROJECT_NAME})
##############################################################


#rosbuild_add_library(${PROJECT_NAME} src/action3.cpp)
#rosbuild_add_executable(traj_controller src/traj_controller.cpp)
#rosbuild_add_executable(cartesian src/cartesian_controller.cpp)
#rosbuild_add_executable(touch src/touch4.cpp)
#rosbuild_add_executable(calib src/calib.cpp)

#target_link_libraries(bosch_arm_node ${PROJECT_NAME})
#target_link_libraries(gcdemo ${PROJECT_NAME})
#common commands for building c++ executables and libraries

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


#message("QiaoZhao Saying")