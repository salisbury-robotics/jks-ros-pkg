#-----------------------------------------------------------------------------
# This is the root CHAI3D CMakeLists file.
#
# Created by Sonny Chan
#	March 2009, Stanford University
#-----------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE Release)
#rosbuild_init()


PROJECT(CHAI3D)

SET(CMAKE_CONFIGURATION_TYPES "Release;ReleaseWithDebInfo" 
    CACHE STRING "Available build configurations." FORCE)
    
#-----------------------------------------------------------------------------
# Output directories.

#TODO: Need to add these system dependencies:
#giflib
# How do I make it so it does NOT overwrite the default ROS Makefile?

#SET(CHAI3D_LIBRARY_PATH "${CHAI3D_BINARY_DIR}/lib")
#SET(CHAI3D_RUNTIME_PATH "${CHAI3D_BINARY_DIR}/bin")
SET(CHAI3D_LIBRARY_PATH "${PROJECT_SOURCE_DIR}/lib")
SET(CHAI3D_RUNTIME_PATH "${PROJECT_SOURCE_DIR}/bin")

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CHAI3D_LIBRARY_PATH})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CHAI3D_LIBRARY_PATH})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CHAI3D_RUNTIME_PATH})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#-----------------------------------------------------------------------------
# Platform-specific definitions and binary directory configuration.

IF(MSVC)
    ADD_DEFINITIONS(-D_MSVC -D_CRT_SECURE_NO_DEPRECATE)
    INCLUDE_DIRECTORIES("external/OpenGL/msvc")
    IF(CMAKE_CL_64)
        LINK_DIRECTORIES("${CHAI3D_SOURCE_DIR}/external/OpenGL/msvc/x64")
        CONFIGURE_FILE("${CHAI3D_SOURCE_DIR}/external/OpenGL/msvc/x64/glut32.dll"
                       "${CHAI3D_BINARY_DIR}/bin/glut32.dll" COPYONLY)
        CONFIGURE_FILE("${CHAI3D_SOURCE_DIR}/external/BASS/dll/x64/bass.dll"
                       "${CHAI3D_BINARY_DIR}/bin/bass.dll" COPYONLY)
    ELSE(CMAKE_CL_64)
        LINK_DIRECTORIES("${CHAI3D_SOURCE_DIR}/external/OpenGL/msvc")
        CONFIGURE_FILE("${CHAI3D_SOURCE_DIR}/external/OpenGL/msvc/glut32.dll"
                       "${CHAI3D_BINARY_DIR}/bin/glut32.dll" COPYONLY)
        CONFIGURE_FILE("${CHAI3D_SOURCE_DIR}/external/BASS/dll/bass.dll"
                       "${CHAI3D_BINARY_DIR}/bin/bass.dll" COPYONLY)
    ENDIF(CMAKE_CL_64)
ENDIF(MSVC)

IF(APPLE)
    ADD_DEFINITIONS(-D_APPLE)
ENDIF(APPLE)

IF(UNIX)
    ADD_DEFINITIONS(-DLINUX -D_ENABLE_DELTA_DEVICE_SUPPORT -DO2)
    INCLUDE_DIRECTORIES("${CHAI3D_SOURCE_DIR}/external/DHD/include")
  IF(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    SET(CHAI3D_LIBPATH "lin-x86_64/release/")
  ELSE()
    SET(CHAI3D_LIBPATH "lin-i686/release/")
  ENDIF()
  LINK_DIRECTORIES("${CHAI3D_SOURCE_DIR}/lib/${CHAI3D_LIBPATH}" "${CHAI3D_SOURCE_DIR}/external/DHD/lib/${CHAI3D_LIBPATH}")
ENDIF(UNIX)

#-----------------------------------------------------------------------------
# Dispatch the build into the proper subdirectories.

INCLUDE_DIRECTORIES("src")

ADD_SUBDIRECTORY(src)
#ADD_SUBDIRECTORY(external)
ADD_SUBDIRECTORY(examples)
