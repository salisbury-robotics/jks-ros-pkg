ADD_EXECUTABLE(25-cubic
    src/25-cubic.cpp
)

TARGET_LINK_LIBRARIES(25-cubic chai3d)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(cubic PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
