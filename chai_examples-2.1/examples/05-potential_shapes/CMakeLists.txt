
rosbuild_add_executable(05-potential_shapes
    05-potential_shapes.cpp
)


# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(devices PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
