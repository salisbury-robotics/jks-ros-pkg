#-----------------------------------------------------------------------------
# CMakeLists file for building the CHAI3D SensAble Phantom driver DLL.
#
# Created by Sonny Chan
#	March 2009, Stanford University
#-----------------------------------------------------------------------------

# First find out where the OpenHaptics/3DTouch API is installed.
FIND_PATH(3DTOUCH_BASE
	NAMES	include/HD/hd.h
	PATHS	"${CMAKE_INSTALL_PREFIX}/SensAble/3DTouch" ENV 3DTOUCH_BASE
)

INCLUDE_DIRECTORIES("${3DTOUCH_BASE}/include" "${3DTOUCH_BASE}/utilities/include")

# Condition on whether this is a 32-bit or 64-bit windows build.
IF(CMAKE_CL_64)
    LINK_DIRECTORIES("${3DTOUCH_BASE}/lib/x64" "${3DTOUCH_BASE}/utilities/lib/x64")
ELSE(CMAKE_CL_64)
    LINK_DIRECTORIES("${3DTOUCH_BASE}/lib/win32" "${3DTOUCH_BASE}/utilities/lib/win32")
ENDIF(CMAKE_CL_64)

ADD_LIBRARY(hdPhantom SHARED
    msvc/StdAfx.cpp
    msvc/hdPhantom.cpp
    msvc/hdPhantom.def
)

TARGET_LINK_LIBRARIES(hdPhantom hd)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC)
    SET_TARGET_PROPERTIES(hdPhantom PROPERTIES PREFIX "../")
ENDIF(MSVC)
