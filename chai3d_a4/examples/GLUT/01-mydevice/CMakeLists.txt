
add_executable(01-mydevice
    01-mydevice.cpp
)


TARGET_LINK_LIBRARIES(01-mydevice
  ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(01-mydevice
chai3d
)
