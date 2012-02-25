//===========================================================================
/*
    This file is part of the CHAI3D visualization and haptics libraries.
    Copyright (C) 2003-2011 by CHAI3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI3D about acquiring a
    Professional Edition License.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 713 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CCHAI3DH
#define CCHAI3DH
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       chai3d.h
    
    \brief    
    <b> CHAI3D </b> \n
    Main Header File.
*/
//===========================================================================

//---------------------------------------------------------------------------
//!     \defgroup   devices  Devices
//---------------------------------------------------------------------------
#include "devices/CCallback.h"
#include "devices/CGenericDevice.h"
#include "devices/CHapticDeviceHandler.h"
#include "devices/CMyCustomDevice.h"

#if defined(WIN32) | defined(WIN64)
#include "devices/CDeltaDevices.h"     
#include "devices/CFalconDevice.h"
#include "devices/CFreedom6SDevice.h"
#include "devices/CPhantomDevices.h"
#include "devices/CVirtualDevice.h"
#endif

#if defined(LINUX)
#include "devices/CDeltaDevices.h"
#endif

#if defined(MACOSX)
#include "devices/CDeltaDevices.h"
#endif


//---------------------------------------------------------------------------
//!     \defgroup   graphics  Graphics 
//---------------------------------------------------------------------------
#include "graphics/CColor.h"
#include "graphics/CDraw3D.h"
#include "graphics/CFog.h"
#include "graphics/CFont.h"
#include "graphics/CImage.h"
#include "graphics/CMacrosGL.h"
#include "graphics/CPrimitives.h"
#include "graphics/CRenderOptions.h"
#include "graphics/CTriangle.h"
#include "graphics/CVertex.h"


//---------------------------------------------------------------------------
//!     \defgroup   materials  Material Properties 
//---------------------------------------------------------------------------
#include "materials/CGenericTexture.h"
#include "materials/CMaterial.h"
#include "materials/CTexture1d.h"
#include "materials/CTexture2d.h"


//---------------------------------------------------------------------------
//!     \defgroup   math  Math 
//---------------------------------------------------------------------------
#include "math/CConstants.h"
#include "math/CMaths.h"
#include "math/CMatrix3d.h"
#include "math/CQuaternion.h"
#include "math/CVector3d.h"


//---------------------------------------------------------------------------
//!     \defgroup   widgets  Widgets
//---------------------------------------------------------------------------
#include "widgets/CBackground.h"
#include "widgets/CBitmap.h"
#include "widgets/CLabel.h"


//---------------------------------------------------------------------------
//!     \defgroup   world  World
//---------------------------------------------------------------------------
#include "world/CGenericObject.h"
#include "world/CMesh.h"
#include "world/CMultiMesh.h"
#include "world/CShapeBox.h"
#include "world/CShapeCylinder.h"
#include "world/CShapeLine.h"
#include "world/CShapeSphere.h"
#include "world/CShapeTorus.h"
#include "world/CWorld.h"


//---------------------------------------------------------------------------
//!     \defgroup   lighting  Lighting Properties 
//---------------------------------------------------------------------------
#include "lighting/CGenericLight.h"
#include "lighting/CDirectionalLight.h"
#include "lighting/CPositionalLight.h"
#include "lighting/CSpotLight.h"
#include "lighting/CShadowMap.h"


//---------------------------------------------------------------------------
//!     \defgroup   tools  Haptic Tools
//---------------------------------------------------------------------------
#include "tools/CGenericTool.h"
#include "tools/CInteractionPoint.h"
#include "tools/CToolCursor.h"
#include "tools/CToolGripper.h"
#include "tools/CToolBody.h"


//---------------------------------------------------------------------------
//!     \defgroup   effects  Haptic Effects
//---------------------------------------------------------------------------
#include "effects/CGenericEffect.h"
#include "effects/CEffectMagnet.h"
#include "effects/CEffectSurface.h"
#include "effects/CEffectStickSlip.h"
#include "effects/CEffectViscosity.h"
#include "effects/CEffectVibration.h"


//---------------------------------------------------------------------------
//!     \defgroup   forces  Force Rendering Algorithms
//---------------------------------------------------------------------------
#include "forces/CGenericForceAlgorithm.h"
#include "forces/CAlgorithmFingerProxy.h"
#include "forces/CAlgorithmPotentialField.h"
#include "forces/CInteractionBasics.h"


//---------------------------------------------------------------------------
//!     \defgroup   collisions  Collision Detection
//---------------------------------------------------------------------------
#include "collisions/CGenericCollision.h"
#include "collisions/CCollisionBasics.h"
#include "collisions/CCollisionBrute.h"
#include "collisions/CCollisionAABB.h"
#include "collisions/CCollisionAABBBox.h"
#include "collisions/CCollisionAABBTree.h"


//---------------------------------------------------------------------------
//!     \defgroup   timers  Timers
//---------------------------------------------------------------------------
#include "timers/CFrequencyCounter.h"
#include "timers/CPrecisionClock.h"
#include "timers/CThread.h"


//---------------------------------------------------------------------------
//!     \defgroup   files  Files
//---------------------------------------------------------------------------
#include "files/CFileImageBMP.h"
#include "files/CFileImageGIF.h"
#include "files/CFileImageJPG.h"
#include "files/CFileImagePNG.h"
#include "files/CFileImagePPM.h"
#include "files/CFileImageRAW.h"
#include "files/CFileModel3DS.h"
#include "files/CFileModelOBJ.h"


//---------------------------------------------------------------------------
//!     \defgroup   extras  Extras
//---------------------------------------------------------------------------
#include "extras/CExtras.h"
#include "extras/CGenericType.h"
#include "extras/CGlobals.h"
#include "extras/CString.h"


//---------------------------------------------------------------------------
//!     \defgroup   resources Resources
//---------------------------------------------------------------------------
#include "resources/CChai3dLogo.h"
#include "resources/CFontCalibri16.h"
#include "resources/CFontCalibri18.h"
#include "resources/CFontCalibri20.h"
#include "resources/CFontCalibri22.h"
#include "resources/CFontCalibri24.h"
#include "resources/CFontCalibri26.h"
#include "resources/CFontCalibri28.h"
#include "resources/CFontCalibri32.h"
#include "resources/CFontMetaCorr36.h"


//---------------------------------------------------------------------------
//!     \defgroup   display  Viewports
//---------------------------------------------------------------------------
#include "display/CCamera.h"

#if defined(WIN32) | defined(WIN64)
#include "display/CViewport.h"
#endif


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
