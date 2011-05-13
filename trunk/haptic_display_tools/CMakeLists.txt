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


set(ROS_BUILD_TYPE Release)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_add_boost_directories()

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_library(haptic_display_tools 
src/HapticDisplay.cpp
src/HapticInstrument.cpp
src/HapticIsosurface.cpp
src/HapticScene.cpp
src/IntervalCollisionDetector.cpp
src/ModifiedMorrisSurface.cpp
src/PointShellIsosurface.cpp
src/WilhelmsenProjection.cpp
)

# Check for SSE
rosbuild_check_for_sse()

TARGET_LINK_LIBRARIES(haptic_display_tools
  chai3d dhd HD
  pthread rt usb-1.0
  GL GLU glut
)
