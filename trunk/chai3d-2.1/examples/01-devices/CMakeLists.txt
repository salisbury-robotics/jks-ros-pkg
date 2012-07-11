
add_executable(01-devices
    01-devices.cpp
)


TARGET_LINK_LIBRARIES(01-devices
  ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(01-devices
chai3d
)
