
add_executable(14-textures
    14-textures.cpp
)


TARGET_LINK_LIBRARIES(14-textures
    ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(14-textures
    chai3d
)
