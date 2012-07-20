add_subdirectory(haptic_sandbox)
#add_subdirectory(tests)

rosbuild_add_executable(user_test
                        main.cpp)

target_link_libraries(user_test
                      haptic_sandbox)
