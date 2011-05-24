cmake_minimum_required (VERSION 2.6)
project (Controller)
set(CMAKE_CXX_FLAGS "-g -Wall")
set(CMAKE_VERBOSE_MAKEFILE on)

link_directories(${Controller_SOURCE_DIR}/../lib)
include_directories(${Controller_SOURCE_DIR}/../include ${Test_SOURCE_DIR}/../src)

add_executable (Controller controller.cpp daq.cpp robot.cpp)
target_link_libraries (Controller s626 pthread rt)

