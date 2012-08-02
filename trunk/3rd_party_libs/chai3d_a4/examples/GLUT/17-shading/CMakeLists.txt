
add_executable(17-shading
    17-shading.cpp
)


TARGET_LINK_LIBRARIES(17-shading
    ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(17-shading
    chai3d
)
