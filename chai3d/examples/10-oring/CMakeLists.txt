ADD_EXECUTABLE(oring
    src/oring.cpp
)

TARGET_LINK_LIBRARIES(oring chai3d_complete)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(oring PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
