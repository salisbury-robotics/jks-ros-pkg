cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rospack(find_elevator_button_new)
SET(CMAKE_VERBOSE_MAKEFILE ON)
SET(CMAKE_C_FLAGS_DEBUG ON)

include_directories(
    ~/lasik/include 
    ~/lasik/external
    ~/lasik/svl/lib/vision
    ~/lasik/external/opencv/include/opencv
     lasik_elevator
     utils)

link_directories(
  ~/lasik/svl/lib/vision
  ~/lasik/svl/lib/_intermediates/base
  ~/lasik/bin
  ~/lasik/external/opencv/lib
  ~/lasik/external/xmlParser
  ~/lasik/external/libsvm
  ~/lasik/external/lbfgs)

SET(LIB_SRC lasik_elevator/elevdata/ElevData.cpp lasik_elevator/buttonDetector/ElevSwod.cpp 
  lasik_elevator/gridfit/ElevGrid.cpp lasik_elevator/gridfit/ElevFit.cpp 
  lasik_elevator/gridfit/ElevInit.cpp lasik_elevator/ocr/ElevOCR.cpp 
  lasik_elevator/ocr/ElevCrop.cpp lasik_elevator/ocr/ElevBin.cpp 
  lasik_elevator/ocr/ElevTesseract.cpp lasik_elevator/hmm/ElevObsSeries.cpp 
  lasik_elevator/hmm/ElevHmmMatrix.cpp lasik_elevator/main/ElevClassifier.cpp)

rosbuild_add_library(elevator ${LIB_SRC} utils/map_pixels.cpp)

list(APPEND ELEVATOR_LIBS elevator svlBase svlVision svlML xmlParser
     cxcore cv highgui cvaux ml svm svlCodeProfiler svlThreadPool svlOptions 
     svlOptimizer svlStatsUtils lbfgs)

rosbuild_add_executable(bin/find_elevator_button nodes/find_elevator_button.cpp)
target_link_libraries(bin/find_elevator_button ${ELEVATOR_LIBS})
