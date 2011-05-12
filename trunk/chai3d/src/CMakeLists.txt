#-----------------------------------------------------------------------------
# CMakeLists file for building the CHAI3D library.
#
# Created by Sonny Chan
#	March 2009, Stanford University
#-----------------------------------------------------------------------------

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


SET(COLLISIONS_GROUP
    collisions/CCollisionAABB.cpp
    collisions/CCollisionAABBBox.cpp
    collisions/CCollisionAABBTree.cpp
    collisions/CCollisionBasics.cpp
    collisions/CCollisionBrute.cpp
    collisions/CCollisionSpheres.cpp
    collisions/CCollisionSpheresGeometry.cpp
    collisions/CGenericCollision.cpp
)

SET(DEVICES_GROUP
    devices/CDeltaDevices.cpp       # should be okay on Mac once we get libs
    devices/CCallback.cpp
    devices/CFreedom6SDevice.cpp
    devices/CGenericDevice.cpp
    devices/CGenericHapticDevice.cpp
    devices/CHapticDeviceHandler.cpp
    devices/CMyCustomDevice.cpp

)

IF(WIN32)
	SET(DEVICES_GROUP ${DEVICES_GROUP}
		devices/CDriverSensoray626.cpp
		devices/CDriverServotogo.cpp
		devices/CFalconDevice.cpp
		devices/CPhantomDevices.cpp
		devices/CVirtualDevice.cpp
		devices/Win626.c		
	)
	SET_SOURCE_FILES_PROPERTIES("devices/Win626.c" PROPERTIES LANGUAGE C)
ENDIF(WIN32)

SET(DISPLAY_GROUP
    display/CViewport.cpp
)

SET(EFFECTS_GROUP
    effects/CEffectMagnet.cpp
    effects/CEffectStickSlip.cpp
    effects/CEffectSurface.cpp
    effects/CEffectVibration.cpp
    effects/CEffectViscosity.cpp
    effects/CGenericEffect.cpp
)

SET(EXTRAS_GROUP
    extras/CExtras.cpp
)

SET(FILES_GROUP
    files/CFileLoader3DS.cpp
    files/CFileLoaderBMP.cpp
    files/CFileLoaderOBJ.cpp
    files/CFileLoaderTGA.cpp
    files/CImageLoader.cpp
    files/CMeshLoader.cpp
)

SET(FORCES_GROUP
    forces/CGenericPointForceAlgo.cpp
    forces/CInteractionBasics.cpp
    forces/CPotentialFieldForceAlgo.cpp
    forces/CProxyPointForceAlgo.cpp
)

SET(GRAPHICS_GROUP
    graphics/CColor.cpp
    graphics/CDraw3D.cpp
    graphics/CGenericTexture.cpp
    graphics/CMacrosGL.cpp
    graphics/CMaterial.cpp
    graphics/CTexture2D.cpp
    graphics/CTriangle.cpp
    graphics/CVertex.cpp
)

SET(MATH_GROUP
    math/CMaths.cpp
    math/CMatrix3d.cpp
    math/CQuaternion.cpp
    math/CString.cpp
    math/CVector3d.cpp
)

SET(SCENEGRAPH_GROUP
    scenegraph/CCamera.cpp
    scenegraph/CGenericObject.cpp
    scenegraph/CLight.cpp
    scenegraph/CMesh.cpp
    scenegraph/CShapeLine.cpp
    scenegraph/CShapeSphere.cpp
    scenegraph/CShapeTorus.cpp
    scenegraph/CWorld.cpp
)

SET(TIMERS_GROUP
    timers/CPrecisionClock.cpp
    timers/CThread.cpp
)

SET(TOOLS_GROUP
    tools/CGeneric3dofPointer.cpp
    tools/CGenericTool.cpp
)

SET(WIDGETS_GROUP
    widgets/CBitmap.cpp
    widgets/CFont.cpp
    widgets/CLabel.cpp
)

SOURCE_GROUP(collisions FILES ${COLLISIONS_GROUP})
SOURCE_GROUP(devices    FILES ${DEVICES_GROUP})
SOURCE_GROUP(display    FILES ${DISPLAY_GROUP})
SOURCE_GROUP(effects    FILES ${EFFECTS_GROUP})
SOURCE_GROUP(extras     FILES ${EXTRAS_GROUP})
SOURCE_GROUP(files      FILES ${FILES_GROUP})
SOURCE_GROUP(forces     FILES ${FORCES_GROUP})
SOURCE_GROUP(graphics   FILES ${GRAPHICS_GROUP})
SOURCE_GROUP(math       FILES ${MATH_GROUP})
SOURCE_GROUP(scenegraph FILES ${SCENEGRAPH_GROUP})
SOURCE_GROUP(timers     FILES ${TIMERS_GROUP})
SOURCE_GROUP(tools      FILES ${TOOLS_GROUP})
SOURCE_GROUP(widgets    FILES ${WIDGETS_GROUP})

ADD_LIBRARY(chai3d
    ${COLLISIONS_GROUP}
    ${DEVICES_GROUP}
    ${DISPLAY_GROUP}
    ${EFFECTS_GROUP}
    ${EXTRAS_GROUP}
    ${FILES_GROUP}
    ${FORCES_GROUP}
    ${GRAPHICS_GROUP}
    ${MATH_GROUP}
    ${SCENEGRAPH_GROUP}
    ${TIMERS_GROUP}
    ${TOOLS_GROUP}
    ${WIDGETS_GROUP}
)

SET_TARGET_PROPERTIES(chai3d PROPERTIES
    DEBUG_POSTFIX   "_d"
)

# Find all the OpenGL frameworks if we're on Mac OS X
IF(APPLE)
    FIND_LIBRARY(OPENGL_LIBRARY OpenGL)
    FIND_LIBRARY(GLUT_LIBRARY GLUT)
    TARGET_LINK_LIBRARIES(chai3d ${OPENGL_LIBRARY} ${GLUT_LIBRARY})
ENDIF(APPLE)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC)
    SET_TARGET_PROPERTIES(chai3d PROPERTIES
        PREFIX      "../"
    )
ENDIF(MSVC)
