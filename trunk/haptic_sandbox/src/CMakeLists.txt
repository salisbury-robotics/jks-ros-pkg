ADD_DEFINITIONS(-fPIC)


add_subdirectory(conversions)
add_subdirectory(haptic_sandbox)
#add_subdirectory(tests)

rosbuild_add_executable(user_test
                        main.cpp
)

target_link_libraries(user_test
chai_conversions
haptic_sandbox
)
