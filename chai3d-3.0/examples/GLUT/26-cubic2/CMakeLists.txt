
add_executable(26-cubic2
    26-cubic2.cpp
)


TARGET_LINK_LIBRARIES(26-cubic2
  chai3d dhd
  pthread rt usb-1.0
  GL GLU glut jpeg png gif
)
