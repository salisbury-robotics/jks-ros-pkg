
add_executable(04-shapes
    04-shapes.cpp
)


TARGET_LINK_LIBRARIES(04-shapes
  ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(04-shapes
chai3d
)
