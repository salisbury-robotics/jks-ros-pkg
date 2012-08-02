
add_executable(15-paint
    15-paint.cpp
)


TARGET_LINK_LIBRARIES(15-paint
    ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(15-paint
    chai3d
)
