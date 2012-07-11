
add_executable(10-oring
    10-oring.cpp
)


TARGET_LINK_LIBRARIES(10-oring
    ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(10-oring
    chai3d
)
