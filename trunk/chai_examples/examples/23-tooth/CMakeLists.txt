ADD_EXECUTABLE(23-tooth
    src/23-tooth.cpp
)

TARGET_LINK_LIBRARIES(23-tooth
  chai3d dhd HD
    pthread rt usb-1.0
      GL GLU glut
      )

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(tooth PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
