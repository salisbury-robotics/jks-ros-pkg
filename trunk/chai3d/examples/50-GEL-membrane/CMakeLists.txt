# GEL projects require sources in external/GEL
SET(GEL_DIRECTORY "${PROJECT_SOURCE_DIR}/modules/GEL")
INCLUDE_DIRECTORIES(${GEL_DIRECTORY})

ADD_EXECUTABLE(50-GEL-membrane
    src/50-GEL-membrane.cpp
    ${GEL_DIRECTORY}/CGELSkeletonLink.cpp
    ${GEL_DIRECTORY}/CGELMesh.cpp
    ${GEL_DIRECTORY}/CGELSkeletonNode.cpp
    ${GEL_DIRECTORY}/CGELWorld.cpp
)

#FIND_LIBRARY(CHAI_LIBRARY chai3d)
#FIND_PATH(CHAI3D_BASE
#  NAMES src/chai3d.h
#  PATHS "~/chai3d-2.1.0" ENV CHAI3D_BASE
#)
#INCLUDE_DIRECTORIES("${CHAI3D_BASE}/src")
#IF(UNIX)
#  ADD_DEFINITIONS(-D_LINUX)
#  IF(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
#    SET(CHAI3D_LIBPATH "lin-x86_64")
#  ELSE()
#    SET(CHAI3D_LIBPATH "lin-i686")
#  ENDIF()
#  LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/lib")
#ENDIF(UNIX)

TARGET_LINK_LIBRARIES(50-GEL-membrane chai3d)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(GEL-membrane PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
