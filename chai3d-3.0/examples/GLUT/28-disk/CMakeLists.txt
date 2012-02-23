
add_executable(28-disk
    28-disk.cpp
)


TARGET_LINK_LIBRARIES(28-disk
  ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(28-disk
    chai3d
)
