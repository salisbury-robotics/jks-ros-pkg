
add_executable(12-polygons
    12-polygons.cpp
)


TARGET_LINK_LIBRARIES(12-polygons
  ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(12-polygons
    chai3d
)
