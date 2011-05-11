ADD_EXECUTABLE(polygons
    src/polygons.cpp
)

TARGET_LINK_LIBRARIES(polygons chai3d_complete)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(polygons PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
