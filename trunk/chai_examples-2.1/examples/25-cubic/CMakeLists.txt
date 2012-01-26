rosbuild_add_executable(25-cubic
    src/25-cubic.cpp
)



# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(cubic PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
