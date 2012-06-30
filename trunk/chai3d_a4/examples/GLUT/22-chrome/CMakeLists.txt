
add_executable(22-chrome
    22-chrome.cpp
)


TARGET_LINK_LIBRARIES(22-chrome
  ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(22-chrome
    chai3d
)

