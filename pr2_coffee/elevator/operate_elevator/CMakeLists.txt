cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(operate_elevator)
SET(CMAKE_VERBOSE_MAKEFILE ON)

#include_directories(../../manipulation/ArmMotionPlanner/include
#                    $ENV{ROS_ROOT}/../ros-pkg/drivers/robot/katana/msg/cpp 
#                    $ENV{ROS_ROOT}/../ros-pkg/drivers/robot/katana/srv/cpp)

#link_directories(/home/stair/ellen/stair/manipulation/ArmMotionPlanner/lib
#                 /home/stair/ellen/stair/3rdparty/inventor-old/inventor/usr/lib)

#link_directories(${PROJECT_SOURCE_DIR}/../../manipulation/ArmMotionPlanner/libmotionplanner
#                 ${PROJECT_SOURCE_DIR}/../../3rdparty/inventor-old/inventor/usr/lib)
#link_directories(${PROJECT_SOURCE_DIR}/../../manipulation/ArmMotionPlanner/lib
#                 ${PROJECT_SOURCE_DIR}/../../3rdparty/inventor-old/inventor/usr/lib)
#list(APPEND ELEVATOR_LIBS motionplanner Inventor InventorXt)

rospack_add_executable(bin/operate_elevator nodes/operate_elevator.cpp)
target_link_libraries(bin/operate_elevator ${ELEVATOR_LIBS})

rospack_add_executable(bin/unwarp_images nodes/unwarp_images.cpp)
target_link_libraries(bin/unwarp_images ${ELEVATOR_LIBS})

rospack_add_executable(bin/begin_operate_elevator nodes/begin_operate_elevator.cpp)
