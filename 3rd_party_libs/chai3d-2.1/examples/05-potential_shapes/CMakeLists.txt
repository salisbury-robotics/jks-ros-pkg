
add_executable(05-potential_shapes
    05-potential_shapes.cpp
)


TARGET_LINK_LIBRARIES(05-potential_shapes
    ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(05-potential_shapes
    chai3d
)
