ADD_EXECUTABLE(12-polygons
    src/12-polygons.cpp
)

TARGET_LINK_LIBRARIES(12-polygons chai3d)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(polygons PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
