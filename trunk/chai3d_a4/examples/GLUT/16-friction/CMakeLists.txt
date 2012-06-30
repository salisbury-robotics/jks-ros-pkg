
add_executable(16-friction
    16-friction.cpp
)


TARGET_LINK_LIBRARIES(16-friction
    ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(16-friction
    chai3d
)
