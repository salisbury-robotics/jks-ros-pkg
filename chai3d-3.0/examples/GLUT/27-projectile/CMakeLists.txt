
add_executable(27-projectile
    27-projectile.cpp
)


TARGET_LINK_LIBRARIES(27-projectile
  ${LINK_DEPENDENCIES}
)

ADD_DEPENDENCIES(27-projectile
    chai3d
)
