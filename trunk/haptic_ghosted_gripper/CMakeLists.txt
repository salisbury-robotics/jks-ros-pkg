cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(Qt4 REQUIRED)
SET(QT_USE_QTMAIN TRUE)
SET(QT_USE_QTCORE TRUE)
SET(QT_USE_QTGUI TRUE)
SET(QT_USE_QTOPENGL TRUE)
INCLUDE( ${QT_USE_FILE} )
ADD_DEFINITIONS(${QT_DEFINITIONS})

include_directories(${QT_INCLUDES} )

SET(my_MOC_HEADERS
src/MyHapticsThread.h)

QT4_WRAP_CPP(my_MOC_SRCS ${my_MOC_HEADERS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(haptic_ghosted_gripper_node
                        ${my_MOC_SRCS}
                        src/MyHapticsThread.cpp
                        src/MyHapticsThread.h
                        src/MyHapticsWidget.cpp
                        src/MyHapticsWidget.h
                        src/cml_conversions.h
                        src/ghosted_gripper_action_server.cpp
                        src/ghosted_gripper_action_server.h
                        src/marker_helpers.cpp
                        src/marker_helpers.h
                        src/main.cpp
                        #src/haptic_ghosted_gripper_node.cpp
                        manifest.xml
                        cfg/Haptics.cfg
                        )


TARGET_LINK_LIBRARIES( haptic_ghosted_gripper_node ${QT_LIBRARIES})


#rosbuild_add_executable(test_haptic_ghosted_gripper
#                        src/test_haptic_ghosted_gripper.cpp
#                        )
