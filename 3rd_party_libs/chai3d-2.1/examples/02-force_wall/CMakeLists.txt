
add_executable(02-force_wall
    02-force_wall.cpp
)


TARGET_LINK_LIBRARIES(02-force_wall
    ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(02-force_wall
    chai3d
)
