
add_executable(13-primitives
    13-primitives.cpp
)


TARGET_LINK_LIBRARIES(13-primitives
    ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(13-primitives
    chai3d
)
