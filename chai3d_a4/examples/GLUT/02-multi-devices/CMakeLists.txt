
add_executable(02-multi-devices
    02-multi-devices.cpp
)


TARGET_LINK_LIBRARIES(02-multi-devices
  ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(02-multi-devices
chai3d
)
