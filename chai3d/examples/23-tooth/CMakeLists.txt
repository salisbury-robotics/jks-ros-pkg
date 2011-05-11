ADD_EXECUTABLE(tooth
    src/tooth.cpp
)

TARGET_LINK_LIBRARIES(tooth chai3d_complete)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(tooth PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
