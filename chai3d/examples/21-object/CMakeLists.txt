ADD_EXECUTABLE(object
    src/object.cpp
)

TARGET_LINK_LIBRARIES(object chai3d_complete)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(object PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
