
ADD_EXECUTABLE(01-mydevice
    01-mydevice.cpp
)


TARGET_LINK_LIBRARIES(01-mydevice
  chai3d dhd
  pthread rt usb-1.0
  GL GLU glut
)
