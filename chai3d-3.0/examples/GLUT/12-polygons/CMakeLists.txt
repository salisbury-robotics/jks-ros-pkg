
add_executable(12-polygons
    12-polygons.cpp
)


TARGET_LINK_LIBRARIES(12-polygons
  chai3d dhd
  pthread rt usb-1.0
  GL GLU glut jpeg png gif
)
