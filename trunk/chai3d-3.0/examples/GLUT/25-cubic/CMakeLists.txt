
add_executable(25-cubic
    25-cubic.cpp
)


TARGET_LINK_LIBRARIES(25-cubic
  chai3d dhd
  pthread rt usb-1.0
  GL GLU glut jpeg png gif
)
