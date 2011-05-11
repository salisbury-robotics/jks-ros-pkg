//===========================================================================
/*
    This file is part of the CHAI 3D visualization and haptics libraries.
    Copyright (C) 2003-#YEAR# by CHAI 3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI 3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI 3D about acquiring a
    Professional Edition License.

    \author:    <http://www.chai3d.org>
    \author:    Francois Conti
    \version    #CHAI_VERSION#
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CCHAI3DH
#define CCHAI3DH
//---------------------------------------------------------------------------

//===========================================================================
/*!
      \file     chai3d.h
      \brief    Includes all the related header files for CHAI 3D.
*/
//===========================================================================

// collisions
#include "collisions/CCollisionAABB.h"
#include "collisions/CCollisionAABBBox.h"
#include "collisions/CCollisionAABBTree.h"
#include "collisions/CCollisionBasics.h"
#include "collisions/CCollisionBrute.h"
#include "collisions/CCollisionSpheres.h"
#include "collisions/CCollisionSpheresGeometry.h"
#include "collisions/CGenericCollision.h"

// devices
#include "devices/CCallback.h"
#include "devices/CGenericDevice.h"
#include "devices/CHapticDeviceHandler.h"
#include "devices/CMyCustomDevice.h"

#if defined(_WIN32)
#include "devices/CDeltaDevices.h"      // move back to above after adding Mac libs
#include "devices/CDriverSensoray626.h"
#include "devices/CDriverServotogo.h"
#include "devices/CFalconDevice.h"
#include "devices/CFreedom6SDevice.h"
#include "devices/CPhantomDevices.h"
#include "devices/CVirtualDevice.h"
#endif

// display
#include "display/CViewport.h"

// effects
#include "effects/CGenericEffect.h"
#include "effects/CEffectMagnet.h"
#include "effects/CEffectSurface.h"
#include "effects/CEffectStickSlip.h"
#include "effects/CEffectViscosity.h"
#include "effects/CEffectVibration.h"

// extras
#include "extras/CExtras.h"
#include "extras/CGlobals.h"

// files
#include "files/CFileLoader3DS.h"
#include "files/CFileLoaderBMP.h"
#include "files/CFileLoaderOBJ.h"
#include "files/CFileLoaderTGA.h"
#include "files/CImageLoader.h"
#include "files/CMeshLoader.h"

// forces
#include "forces/CGenericPointForceAlgo.h"
#include "forces/CPotentialFieldForceAlgo.h"
#include "forces/CProxyPointForceAlgo.h"
#include "forces/CInteractionBasics.h"

// graphics
#include "graphics/CColor.h"
#include "graphics/CDraw3D.h"
#include "graphics/CGenericTexture.h"
#include "graphics/CMacrosGL.h"
#include "graphics/CMaterial.h"
#include "graphics/CTexture2D.h"
#include "graphics/CTriangle.h"
#include "graphics/CVertex.h"

// math
#include "math/CConstants.h"
#include "math/CMaths.h"
#include "math/CString.h"
#include "math/CMatrix3d.h"
#include "math/CQuaternion.h"
#include "math/CVector3d.h"

// scenegraph
#include "scenegraph/CCamera.h"
#include "scenegraph/CGenericObject.h"
#include "scenegraph/CLight.h"
#include "scenegraph/CMesh.h"
#include "scenegraph/CShapeLine.h"
#include "scenegraph/CShapeSphere.h"
#include "scenegraph/CShapeTorus.h"
#include "scenegraph/CWorld.h"

// timers
#include "timers/CPrecisionClock.h"
#include "timers/CThread.h"

// tools
#include "tools/CGeneric3dofPointer.h"
#include "tools/CGenericTool.h"

// widgets
#include "widgets/CBitmap.h"
#include "widgets/CFont.h"
#include "widgets/CLabel.h"

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
