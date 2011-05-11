#-----------------------------------------------------------------------------
# CMakeLists file for building the CHAI3D Novint Falcon driver DLL.
#
# Created by Sonny Chan
#	March 2009, Stanford University
#-----------------------------------------------------------------------------

INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/include")
LINK_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/lib")

ADD_LIBRARY(hdFalcon SHARED
    msvc/StdAfx.cpp
    msvc/hdFalcon.cpp
    msvc/hdFalcon.def
)

TARGET_LINK_LIBRARIES(hdFalcon hdl)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC)
    SET_TARGET_PROPERTIES(hdFalcon PROPERTIES PREFIX "../")
ENDIF(MSVC)
