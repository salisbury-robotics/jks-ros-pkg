rosbuild_add_executable(21-object
    src/21-object.cpp
)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(object PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
