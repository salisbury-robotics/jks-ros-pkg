
add_executable(21-object
    21-object.cpp
)


TARGET_LINK_LIBRARIES(21-object
  ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(21-object
    chai3d
)

