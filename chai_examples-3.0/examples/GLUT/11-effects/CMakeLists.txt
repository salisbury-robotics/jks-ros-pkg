
ADD_EXECUTABLE(11-effects
    11-effects.cpp
)


TARGET_LINK_LIBRARIES(11-effects
  chai3d dhd
  pthread rt usb-1.0
  GL GLU glut
)
