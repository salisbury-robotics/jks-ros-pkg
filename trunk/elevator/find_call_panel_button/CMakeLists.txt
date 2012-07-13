cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(find_call_panel_button)
SET(CMAKE_VERBOSE_MAKEFILE ON)
SET(CMAKE_C_FLAGS_DEBUG ON)

include_directories(/home/master/lasik/include 
                    /home/master/lasik/external
		    /home/master/lasik/svl/lib/vision)
link_directories(/home/master/lasik/external/xmlParser
                 /home/master/lasik/external/lbfgs 
                 /home/master/lasik/bin
		 /home/master/lasik/svl/lib/vision/
		 /home/master/lasik/svl/lib/base/)

rospack_add_library(lib_find_call_panel_button
  SvlClassify/svlElevatorClassifier.cpp 
  SvlClassify/svlElevatorOverlayer.cpp
  SvlClassify/CallButtonEnhancer.cpp)

list(APPEND FIND_CALL_PANEL_BUTTON_LIBS svlBase svlVision svlPGM svlML
                             lib_find_call_panel_button lbfgs xmlParser svlLogger svlCodeProfiler)#  svlObjectList)

add_executable(bin/find_call_panel_button nodes/find_call_panel_button.cpp)
target_link_libraries(bin/find_call_panel_button ${FIND_CALL_PANEL_BUTTON_LIBS})
