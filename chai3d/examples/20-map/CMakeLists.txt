ADD_EXECUTABLE(20-map
    src/20-map.cpp
)

TARGET_LINK_LIBRARIES(20-map chai3d)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(map PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
