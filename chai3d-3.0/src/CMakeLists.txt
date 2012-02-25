#-----------------------------------------------------------------------------
# CMakeLists file for building the CHAI3D library.
#
# Created by Sonny Chan, March 2009
# Edited by Adam Leeper for Chai3d 3.0-alpha
#	January 2012, Stanford University
#-----------------------------------------------------------------------------
SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)

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
    #collisions/CCollisionSpheres.cpp
    #collisions/CCollisionSpheresGeometry.cpp
    collisions/CGenericCollision.cpp
)

SET(DEVICES_GROUP
    devices/CCallback.cpp
    devices/CDeltaDevices.cpp       # should be okay on Mac once we get libs
    devices/CFalconDevice.cpp
    devices/CFreedom6SDevice.cpp
    devices/CGenericDevice.cpp
    devices/CGenericHapticDevice.cpp
    devices/CHapticDeviceHandler.cpp
    devices/CMyCustomDevice.cpp
    devices/CPhantomDevices.cpp
    devices/CVirtualDevice.cpp
)

#IF(WIN32)
#	SET(DEVICES_GROUP ${DEVICES_GROUP}
#		devices/CDriverSensoray626.cpp
#		devices/CDriverServotogo.cpp
#		devices/CFalconDevice.cpp
#		devices/CPhantomDevices.cpp
#		devices/CVirtualDevice.cpp
#		devices/Win626.c		
#	)
#	SET_SOURCE_FILES_PROPERTIES("devices/Win626.c" PROPERTIES LANGUAGE C)
#ENDIF(WIN32)

SET(DISPLAY_GROUP
    display/CCamera.cpp
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
    extras/CString.cpp
)

SET(FILES_GROUP
    files/CFileImageBMP.cpp
    files/CFileImageGIF.cpp
    files/CFileImageJPG.cpp
    files/CFileImagePNG.cpp
    files/CFileImagePPM.cpp
    files/CFileImageRAW.cpp
    files/CFileModel3DS.cpp
    files/CFileModelOBJ.cpp
)

SET(FORCES_GROUP
    forces/CAlgorithmFingerProxy.cpp
    forces/CAlgorithmPotentialField.cpp
    forces/CGenericForceAlgorithm.cpp
    forces/CInteractionBasics.cpp
)

SET(GRAPHICS_GROUP
    graphics/CColor.cpp
    graphics/CDraw3D.cpp
    graphics/CFog.cpp
    graphics/CFont.cpp
    graphics/CImage.cpp
    graphics/CMacrosGL.cpp
    graphics/CPrimitives.cpp    
    graphics/CTriangle.cpp
    graphics/CVertex.cpp
    graphics/GLee.cpp
)

SET(LIGHTING_GROUP
    lighting/CDirectionalLight.cpp
    lighting/CGenericLight.cpp
    lighting/CPositionalLight.cpp
    lighting/CShadowMap.cpp
    lighting/CSpotLight.cpp
)

SET(MATERIALS_GROUP
    materials/CGenericTexture.cpp
    materials/CMaterial.cpp
    materials/CTexture1d.cpp
    materials/CTexture2d.cpp
)

SET(MATH_GROUP
    math/CMaths.cpp
    math/CMatrix3d.cpp
    math/CQuaternion.cpp
    math/CVector3d.cpp
)

SET(TIMERS_GROUP
    timers/CFrequencyCounter.cpp
    timers/CPrecisionClock.cpp
    timers/CThread.cpp
)

SET(TOOLS_GROUP
    tools/CGenericTool.cpp
    tools/CInteractionPoint.cpp
    tools/CToolCursor.cpp
    tools/CToolGripper.cpp
    tools/CToolBody.cpp
)

SET(WIDGETS_GROUP
    widgets/CBackground.cpp
    widgets/CBitmap.cpp
    widgets/CLabel.cpp
)

SET(WORLD_GROUP
    world/CGenericObject.cpp
    world/CMesh.cpp
    world/CMultiMesh.cpp
    world/CShapeBox.cpp
    world/CShapeCylinder.cpp
    world/CShapeLine.cpp
    world/CShapeSphere.cpp
    world/CShapeTorus.cpp
    world/CWorld.cpp
)

SOURCE_GROUP(collisions FILES ${COLLISIONS_GROUP})
SOURCE_GROUP(devices    FILES ${DEVICES_GROUP})
SOURCE_GROUP(display    FILES ${DISPLAY_GROUP})
SOURCE_GROUP(effects    FILES ${EFFECTS_GROUP})
SOURCE_GROUP(extras     FILES ${EXTRAS_GROUP})
SOURCE_GROUP(files      FILES ${FILES_GROUP})
SOURCE_GROUP(forces     FILES ${FORCES_GROUP})
SOURCE_GROUP(graphics   FILES ${GRAPHICS_GROUP})
SOURCE_GROUP(lighting   FILES ${LIGHTING_GROUP})
SOURCE_GROUP(materials   FILES ${MATERIALS_GROUP})
SOURCE_GROUP(math       FILES ${MATH_GROUP})
SOURCE_GROUP(timers     FILES ${TIMERS_GROUP})
SOURCE_GROUP(tools      FILES ${TOOLS_GROUP})
SOURCE_GROUP(widgets    FILES ${WIDGETS_GROUP})
SOURCE_GROUP(world      FILES ${WORLD_GROUP})

ADD_LIBRARY(chai3d
    ${COLLISIONS_GROUP}
    ${DEVICES_GROUP}
    ${DISPLAY_GROUP}
    ${EFFECTS_GROUP}
    ${EXTRAS_GROUP}
    ${FILES_GROUP}
    ${FORCES_GROUP}
    ${GRAPHICS_GROUP}
    ${LIGHTING_GROUP}
    ${MATH_GROUP}
    ${MATERIALS_GROUP}
    ${TIMERS_GROUP}
    ${TOOLS_GROUP}
    ${WIDGETS_GROUP}
    ${WORLD_GROUP}
)

SET_TARGET_PROPERTIES(chai3d PROPERTIES
    DEBUG_POSTFIX   "-dbg"
)

# Find all the OpenGL frameworks if we're on Mac OS X
IF(APPLE)
    FIND_LIBRARY(OPENGL_LIBRARY OpenGL)
    FIND_LIBRARY(GLUT_LIBRARY GLUT)
    TARGET_LINK_LIBRARIES(chai3d ${OPENGL_LIBRARY} ${GLUT_LIBRARY})
ENDIF(APPLE)

TARGET_LINK_LIBRARIES(chai3d 
  #${CHAI_LINK_LIBRARIES}
)

# This is a BAD HACK for MSVC projects to put their targets in the right place!
IF(MSVC)
    SET_TARGET_PROPERTIES(chai3d PROPERTIES
        PREFIX      "../"
    )
ENDIF(MSVC)
