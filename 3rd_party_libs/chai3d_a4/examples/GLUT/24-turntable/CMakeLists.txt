
add_executable(24-turntable
    24-turntable.cpp
)


TARGET_LINK_LIBRARIES(24-turntable
  ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(24-turntable
    chai3d
)

