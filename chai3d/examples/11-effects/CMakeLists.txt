ADD_EXECUTABLE(effects
    src/effects.cpp
)

TARGET_LINK_LIBRARIES(effects chai3d_complete)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(effects PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
