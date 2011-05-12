#-----------------------------------------------------------------------------
# CMakeLists file for building the CHAI3D external driver DLLs.
#
# Created by Sonny Chan
#	March 2009, Stanford University
#-----------------------------------------------------------------------------

IF(WIN32)

    OPTION(BUILD_HDFALCON "Build hdFalcon.dll" OFF)
    IF(BUILD_HDFALCON)
        ADD_SUBDIRECTORY(hdFalcon)
    ENDIF(BUILD_HDFALCON)
    
    OPTION(BUILD_HDPHANTOM "Build hdPhantom.dll" OFF)
    IF(BUILD_HDPHANTOM)
        ADD_SUBDIRECTORY(hdPhantom)
    ENDIF(BUILD_HDPHANTOM)
    
ENDIF(WIN32)
