#-----------------------------------------------------------------------------
# This is the root CHAI3D CMakeLists file.
#
# Created by Sonny Chan
#	March 2009, Stanford University
#-----------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(CHAI3D)

SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" 
    CACHE STRING "Available build configurations." FORCE)
    
#-----------------------------------------------------------------------------
# Output directories.

SET(CHAI3D_LIBRARY_PATH "${CHAI3D_BINARY_DIR}/lib")
SET(CHAI3D_RUNTIME_PATH "${CHAI3D_BINARY_DIR}/bin")

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CHAI3D_LIBRARY_PATH})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CHAI3D_LIBRARY_PATH})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CHAI3D_RUNTIME_PATH})

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
    ADD_DEFINITIONS(-D_LINUX)
ENDIF(UNIX)

#-----------------------------------------------------------------------------
# Dispatch the build into the proper subdirectories.

INCLUDE_DIRECTORIES("src")

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(external)
ADD_SUBDIRECTORY(examples)
