# Turntable demo requires BASS libraries for audio support.
INCLUDE_DIRECTORIES("${CHAI3D_SOURCE_DIR}/external/BASS/include")
IF(CMAKE_CL_64)
    LINK_DIRECTORIES("${CHAI3D_SOURCE_DIR}/external/BASS/lib/msvc/x64")
ELSE(CMAKE_CL_64)
    LINK_DIRECTORIES("${CHAI3D_SOURCE_DIR}/external/BASS/lib/msvc")
ENDIF(CMAKE_CL_64)

ADD_EXECUTABLE(turntable
    src/turntable.cpp
)

TARGET_LINK_LIBRARIES(turntable chai3d_complete)

IF(MSVC)
    TARGET_LINK_LIBRARIES(turntable bass_msvc)
ENDIF(MSVC)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(turntable PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
