
add_executable(20-map
    20-map.cpp
)


TARGET_LINK_LIBRARIES(20-map
  ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(20-map
    chai3d
)

