
cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(find_elevator_button)
SET(CMAKE_VERBOSE_MAKEFILE ON)
SET(CMAKE_C_FLAGS_DEBUG ON)

include_directories(/home/acoates/lasik/include 
                    /home/acoates/lasik/external
		    /home/acoates/lasik/svl/lib/vision)
link_directories(/home/acoates/lasik/external/xmlParser
                 /home/acoates/lasik/external/lbfgs 
                 /home/acoates/lasik/bin
		 /home/acoates/lasik/svl/lib/vision/
		 /home/acoates/lasik/svl/lib/base/)

rospack_add_library(lib_find_buttons lib_find_buttons/EMGridFit/EMGridFit.cpp
                    lib_find_buttons/HMMLabel/HMMLabel.cpp 
                    lib_find_buttons/LabelClassify/label_classify.cpp 
		    lib_find_buttons/SvlClassify/svlElevatorClassifier.cpp 
		    lib_find_buttons/SvlClassify/svlElevatorOverlayer.cpp
		    lib_find_buttons/SvlClassify/CallButtonEnhancer.cpp)

list(APPEND FIND_BUTTON_LIBS svlBase svlVision svlPGM svlML
                             lib_find_buttons lbfgs xmlParser svlLogger svlCodeProfiler)#  svlObjectList)

add_executable(bin/find_elevator_button nodes/find_elevator_button.cpp)
target_link_libraries(bin/find_elevator_button ${FIND_BUTTON_LIBS})

add_executable(bin/EMGridFit_main lib_find_buttons/EMGridFit/EMGridFit_main.cpp)
target_link_libraries(bin/EMGridFit_main ${FIND_BUTTON_LIBS})

#add_executable(bin/HMMLabel_main lib_find_buttons/HMMLabel/HMMLabel_main.cpp)
#target_link_libraries(bin/HMMLabel_main ${FIND_BUTTON_LIBS})
