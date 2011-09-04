ADD_EXECUTABLE(11-effects
    src/11-effects.cpp
)

TARGET_LINK_LIBRARIES(11-effects
  chai3d dhd HD
    pthread rt usb-1.0
      GL GLU glut
      )

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(effects PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
