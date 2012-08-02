
add_executable(05-fonts
    05-fonts.cpp
)


TARGET_LINK_LIBRARIES(05-fonts
  ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(05-fonts
chai3d
)
