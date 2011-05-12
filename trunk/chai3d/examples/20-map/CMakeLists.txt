ADD_EXECUTABLE(map
    src/map.cpp
)

TARGET_LINK_LIBRARIES(map chai3d_complete)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(map PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
