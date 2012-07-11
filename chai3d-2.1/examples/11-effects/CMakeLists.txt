
add_executable(11-effects
    11-effects.cpp
)


TARGET_LINK_LIBRARIES(11-effects
 ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(11-effects
    chai3d
)
