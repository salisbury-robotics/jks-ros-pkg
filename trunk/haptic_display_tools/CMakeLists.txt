cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(Qt4 REQUIRED)
SET(QT_USE_QTMAIN TRUE)
SET(QT_USE_QTCORE TRUE)
SET(QT_USE_QTGUI TRUE)
SET(QT_USE_QTOPENGL TRUE)
INCLUDE( ${QT_USE_FILE} )

include_directories(${QT_INCLUDES} )

include_directories(${PROJECT_SOURCE_DIR}/include/haptic_display_tools)
include_directories(${PROJECT_SOURCE_DIR}/include/haptic_display_tools/Common)
include_directories(${PROJECT_SOURCE_DIR}/include/haptic_display_tools/Graphics)
include_directories(${PROJECT_SOURCE_DIR}/include/haptic_display_tools/Haptics)


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
#ADD_DEFINITIONS(-fPIC)


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
src/Haptics/HapticDisplay.cpp
src/Haptics/HapticScene.cpp
src/Haptics/IntervalCollisionDetector.cpp
src/Haptics/HapticInstrument.cpp
src/Haptics/HapticIsosurface.cpp
src/Haptics/ModifiedMorrisSurface.cpp
src/Haptics/PointShellIsosurface.cpp
src/Haptics/WilhelmsenProjection.cpp

src/Graphics/Mesh.cpp
src/Graphics/MeshRepository.cpp
src/Graphics/ProxyGeometry.cpp
src/Graphics/ShaderPackage.cpp
src/Graphics/SphericalCamera.cpp
src/Graphics/TrackballCamera.cpp
src/Graphics/TransferFunction.cpp
src/Graphics/VolumeRenderer.cpp

src/Common/Parameters.cpp
src/Common/VolumeCutter.cpp
#src/Common/Sampler.cpp
src/Common/VolumeSampler.cpp
src/Common/PointSampler.cpp

manifest.xml
)

#IF( CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" )
#  SET_TARGET_PROPERTIES(haptic_display_tools PROPERTIES COMPILE_FLAGS "-fPIC")
#ENDIF( CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" )

#rosbuild_add_library(graphic_display_tools
#)


#rosbuild_add_library(common_display_tools 
#src/Common/Parameters.cpp
#src/Common/VolumeCutter.cpp
#src/Common/VolumeSampler.cpp
#)

# Check for SSE
rosbuild_check_for_sse()

#TARGET_LINK_LIBRARIES( haptic_display_tools  )
#TARGET_LINK_LIBRARIES( graphic_display_tools  chai3d dhd HD pthread rt usb-1.0 GL GLU glut )
#TARGET_LINK_LIBRARIES( common_display_tools   chai3d dhd HD pthread rt usb-1.0 GL GLU glut )
