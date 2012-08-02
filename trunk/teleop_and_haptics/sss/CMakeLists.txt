cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(Qt4 REQUIRED)
SET(QT_USE_QTMAIN TRUE)
SET(QT_USE_QTCORE TRUE)
SET(QT_USE_QTGUI TRUE)
SET(QT_USE_QTOPENGL TRUE)
INCLUDE( ${QT_USE_FILE} )
ADD_DEFINITIONS(${QT_DEFINITIONS})

include_directories(${QT_INCLUDES} )

SET( my_PROGRAM_SOURCES 
Source/MyDeviceDetailsWidget.cpp
Source/MyHapticDevicesModel.cpp
Source/MyHapticDevicesWidget.cpp
Source/MyHapticsGLWidget.cpp
Source/MyHapticsThread.cpp
Source/MyImageGLWidget.cpp
Source/MyLabellingEditor.cpp
Source/MyLabelsGLWidget.cpp
Source/MyLabelsViewWidget.cpp
Source/MyLabelsWidget.cpp
Source/MyLabelVolumeFilterModel.cpp
Source/MyMainWindow.cpp
Source/MyMaterialEditor.cpp
Source/MyMeshDetailsWidget.cpp
Source/MyMeshesModel.cpp
Source/MyMeshesWidget.cpp
Source/MyMeshGLWidget.cpp
Source/MyProgressReporter.cpp
Source/MyRendererGLWidget.cpp
Source/MyRendererPresetsWidget.cpp
Source/MySliceGLWidget.cpp
Source/MyVisualizationWidget.cpp
Source/MyVolumeDetailsWidget.cpp
Source/MyVolumeHistogramWidget.cpp
Source/MyVolumeImageWidget.cpp
Source/MyVolumesModel.cpp
Source/MyVolumesWidget.cpp
Source/s_MyRendererTFEditor_CH.cpp
Source/s_MyRendererTFEditor.cpp
Source/s_MyRendererTFEditor_RS.cpp
Source/s_MyRendererTFEditor_TFE.cpp
)

SET(my_MOC_HEADERS 
Source/MyDeviceDetailsWidget.h
Source/MyHapticDevicesModel.h
Source/MyHapticDevicesWidget.h
Source/MyHapticsGLWidget.h
Source/MyHapticsThread.h
Source/MyImageGLWidget.h
Source/MyLabellingEditor.h
Source/MyLabelsGLWidget.h
Source/MyLabelsViewWidget.h
Source/MyLabelsWidget.h
Source/MyLabelVolumeFilterModel.h
Source/MyMainWindow.h
Source/MyMaterialEditor.h
Source/MyMeshDetailsWidget.h
Source/MyMeshesModel.h
Source/MyMeshesWidget.h
Source/MyMeshGLWidget.h
Source/MyProgressReporter.h
Source/MyRendererGLWidget.h
Source/MyRendererPresetsWidget.h
Source/MySliceGLWidget.h
Source/MyVisualizationWidget.h
Source/MyVolumeDetailsWidget.h
Source/MyVolumeHistogramWidget.h
Source/MyVolumeImageWidget.h
Source/MyVolumesModel.h
Source/MyVolumesWidget.h
Source/s_MyRendererTFEditor_CH.h
Source/s_MyRendererTFEditor.h
Source/s_MyRendererTFEditor_RS.h
Source/s_MyRendererTFEditor_TFE.h
)

SET(my_FORMS Forms/AboutDialog.ui)

QT4_WRAP_UI(my_FORMS_HEADERS ${my_FORMS})
QT4_WRAP_CPP(my_MOC_SRCS ${my_MOC_HEADERS})
QT4_ADD_RESOURCES(my_RESOURCES Simulation.qrc)



include_directories(${PROJECT_SOURCE_DIR} )
include_directories(${PROJECT_SOURCE_DIR}/build )


SET(ITK_DIR /usr/include/InsightToolkit)

include_directories(${ITK_DIR} 
                    ${ITK_DIR}/Algorithms 
                    ${ITK_DIR}/BasicFilters 
                    ${ITK_DIR}/Common 
                    ${ITK_DIR}/IO 
                    ${ITK_DIR}/Utilities 
                    ${ITK_DIR}/Utilities/vxl/core 
                    ${ITK_DIR}/Utilities/vxl/vcl 
                    ${ITK_DIR}/gdcm/src
)

SET( GDCM_DIR /usr/include/gdcm-2.0)
include_directories( ${GDCM_DIR} )


set(ROS_BUILD_TYPE Release)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_add_boost_directories()

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_executable(sss
${my_FORMS}
${my_MOC_SRCS}
${my_RESOURCES}
Source/main.cpp

${my_PROGRAM_SOURCES}
Data/DicomReader.cpp
Data/LabelsVolumeGenerator.cpp
Data/VolumeData.cpp
Data/VolumeRepository.cpp
)


# Check for SSE
rosbuild_check_for_sse()

TARGET_LINK_LIBRARIES( sss ${QT_LIBRARIES} -lITKIO -litksys )
#TARGET_LINK_LIBRARIES( graphic_display_tools  chai3d dhd HD pthread rt usb-1.0 GL GLU glut )
#TARGET_LINK_LIBRARIES( common_display_tools   chai3d dhd HD pthread rt usb-1.0 GL GLU glut )
