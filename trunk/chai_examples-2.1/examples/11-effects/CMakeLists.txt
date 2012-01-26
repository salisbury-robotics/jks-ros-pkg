rosbuild_add_executable(11-effects
    src/11-effects.cpp
)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(effects PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
