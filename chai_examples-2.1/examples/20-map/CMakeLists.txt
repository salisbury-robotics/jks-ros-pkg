rosbuild_add_executable(20-map
    src/20-map.cpp
)


# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(map PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
