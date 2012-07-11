
add_executable(25-cubic
    25-cubic.cpp
)


TARGET_LINK_LIBRARIES(25-cubic
  ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(25-cubic
    chai3d
)

