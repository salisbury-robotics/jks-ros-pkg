rosbuild_add_executable(23-tooth
    src/23-tooth.cpp
)


# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(tooth PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
