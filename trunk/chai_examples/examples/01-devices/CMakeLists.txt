
ADD_EXECUTABLE(01-devices
    src/01-devices.cpp
)


TARGET_LINK_LIBRARIES(01-devices
  chai3d dhd HD
  pthread rt usb-1.0
  GL GLU glut
)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(devices PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
