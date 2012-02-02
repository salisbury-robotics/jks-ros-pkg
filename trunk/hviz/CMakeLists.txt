cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#FIND_PATH(CHAI3D_BASE
#  NAMES src/chai3d.h
#  PATHS "~/libraries/chai3d-2.1.0" ENV CHAI3D_BASE
#)
#INCLUDE_DIRECTORIES("${CHAI3D_BASE}/src")
#IF(UNIX)
#  ADD_DEFINITIONS(-D_LINUX)
#  IF(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
#    SET(CHAI3D_LIBPATH "lin-x86_64")
#  ELSE()
#    SET(CHAI3D_LIBPATH "lin-i686")
#  ENDIF()
#  LINK_DIRECTORIES("${CHAI3D_BASE}/lib/${CHAI3D_LIBPATH}" "${CHAI3D_BASE}/external/DHD/lib/${CHAI3D_LIBPATH}")
#ENDIF(UNIX)



# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

find_package (VTK REQUIRED)
include (${VTK_USE_FILE})


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()


rosbuild_add_executable(haptics_node src/main.cpp
#src/point_rendering.cpp
#src/point_rendering.h
src/PointCloudObject.cpp
src/PointCloudObject_shapes.cpp
#src/depth_sensor.cpp
src/cplane.cpp
#src/kdtree.cpp
src/helpers.cpp
src/CThreadWithArgument.h
cfg/Haptics.cfg
manifest.xml
)


#TARGET_LINK_LIBRARIES(haptics_node
  #HD chai3d dhd
  #pthread rt usb-1.0
  #GL GLU glut
#)
