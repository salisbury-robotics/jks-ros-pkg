
ADD_EXECUTABLE(05-potential_shapes
    05-potential_shapes.cpp
)


TARGET_LINK_LIBRARIES(05-potential_shapes
  chai3d dhd
  pthread rt usb-1.0
  GL GLU glut
)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(devices PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
