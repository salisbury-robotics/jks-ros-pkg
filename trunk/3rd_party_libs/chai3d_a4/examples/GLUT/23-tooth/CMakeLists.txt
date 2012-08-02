
add_executable(23-tooth
    23-tooth.cpp
)


TARGET_LINK_LIBRARIES(23-tooth
  ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(23-tooth
    chai3d
)

