
add_executable(03-analytics
    03-analytics.cpp
)


TARGET_LINK_LIBRARIES(03-analytics
  ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(03-analytics
chai3d
)
