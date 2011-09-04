ADD_EXECUTABLE(21-object
    src/21-object.cpp
)

TARGET_LINK_LIBRARIES(21-object 
  chai3d dhd HD
    pthread rt usb-1.0
      GL GLU glut
      )

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC OR APPLE)
    SET_TARGET_PROPERTIES(object PROPERTIES PREFIX "../")
ENDIF(MSVC OR APPLE)
