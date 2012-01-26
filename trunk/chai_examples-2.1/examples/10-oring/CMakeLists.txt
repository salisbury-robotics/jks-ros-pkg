rosbuild_add_executable(10-oring
    src/10-oring.cpp
)


# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(oring PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
