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
#libgif-dev lib3ds-dev
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
    ADD_DEFINITIONS(-DMACOSX -D_ENABLE_DELTA_DEVICE_SUPPORT -DO2)
    INCLUDE_DIRECTORIES("${CHAI3D_SOURCE_DIR}/external/libpng/include")
    INCLUDE_DIRECTORIES("${CHAI3D_SOURCE_DIR}/external/libjpeg/include")
    INCLUDE_DIRECTORIES("${CHAI3D_SOURCE_DIR}/external/giflib/include")
    INCLUDE_DIRECTORIES("${CHAI3D_SOURCE_DIR}/external/DHD/include")
    LINK_DIRECTORIES("${CHAI3D_SOURCE_DIR}/lib"
                     "${CHAI3D_SOURCE_DIR}/external/libpng/lib/mac/Release"
                     "${CHAI3D_SOURCE_DIR}/external/libjpeg/lib/mac/Release"
                     "${CHAI3D_SOURCE_DIR}/external/giflib/lib/mac/Release"
                     "${CHAI3D_SOURCE_DIR}/external/DHD/lib/mac")
    SET(LINK_DEPENDENCIES "chai3d -ldhd -lpthread -ljpeg -lpng -lgif"
                          "-framework OpenGL -framework GLUT"
                          "-framework CoreFoundation -framework IOKit")
ELSEIF(UNIX)
    ADD_DEFINITIONS(-DLINUX -fPIC -DO2)
    #ADD_DEFINITIONS(-DC_ENABLE_CUSTOM_DEVICE_SUPPORT )
    ADD_DEFINITIONS(-DC_ENABLE_DELTA_DEVICE_SUPPORT )
    #ADD_DEFINITIONS(-DC_ENABLE_PHANTOM_DEVICE_SUPPORT )
    #SET(CHAI_LINK_LIBRARIES "-lHD")
    INCLUDE_DIRECTORIES("${CHAI3D_SOURCE_DIR}/internal/DHD/include")
    INCLUDE_DIRECTORIES("${CHAI3D_SOURCE_DIR}/internal/lib3ds/include")
    INCLUDE_DIRECTORIES("${CHAI3D_SOURCE_DIR}/external/BASS/include")
  IF(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    SET(CHAI3D_LIBPATH "lin-x86_64/")
  ELSE()
    SET(CHAI3D_LIBPATH "lin-i686/")
  ENDIF()
  LINK_DIRECTORIES("${CHAI3D_SOURCE_DIR}/lib")
  LINK_DIRECTORIES("${CHAI3D_SOURCE_DIR}/internal/DHD/lib/${CHAI3D_LIBPATH}")
  LINK_DIRECTORIES("${CHAI3D_SOURCE_DIR}/internal/lib3ds/lib/release/${CHAI3D_LIBPATH}")
  IF(C_ENABLE_PHANTOM_DEVICE_SUPPORT)
    SET(LINK_DEPENDENCIES "-lchai3d -ldhd -lpthread -lrt -lusb-1.0 -lGL -lGLU -lglut -ljpeg -lpng -lgif -l3ds -lHD")
  ELSE()
    SET(LINK_DEPENDENCIES "-lchai3d -ldhd -lpthread -lrt -lusb-1.0 -lGL -lGLU -lglut -ljpeg -lpng -lgif -l3ds")
  ENDIF(C_ENABLE_PHANTOM_DEVICE_SUPPORT)
ENDIF(APPLE)

#-----------------------------------------------------------------------------
# Dispatch the build into the proper subdirectories.

INCLUDE_DIRECTORIES("src")

ADD_SUBDIRECTORY(src)
#ADD_SUBDIRECTORY(external)
ADD_SUBDIRECTORY(examples)
