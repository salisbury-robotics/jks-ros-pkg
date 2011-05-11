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
#ifndef CInteractionBasicsH
#define CInteractionBasicsH
//---------------------------------------------------------------------------
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
#include "graphics/CVertex.h"
#include "graphics/CMaterial.h"
#include <vector>
//---------------------------------------------------------------------------
using std::vector;
class cGenericObject;
//---------------------------------------------------------------------------

//===========================================================================
/*!
      \file     CInteractionBasics.h
      \struct   cInteractionEvent
      \brief    cInteractionEvent stores all information related to the
      intersection between a point and an object.
*/
//===========================================================================
struct cInteractionEvent
{
    //! Pointer to the interaction object.
    cGenericObject* m_object;

    //! Is the pointer located inside the object
    bool m_isInside;

    //! Position of the interaction point in reference to the objects coordinate frame (local coordinates).
    cVector3d m_localPos;

    //! Resulting force in local coordinates.
    cVector3d m_localForce;

    //! Nearest point to the object surface in local coordinates
    cVector3d m_localSurfacePos;

    //! initialize all data
    void clear()
    {
        m_object    = NULL;
        m_isInside    = false;
        m_localPos.zero();
        m_localForce.zero();
        m_localSurfacePos.zero();
    }
};


//===========================================================================
/*!
      \class    cInteractionRecorder
      \brief    cInteractionRecorder stores a list of interaction events.
*/
//===========================================================================
class cInteractionRecorder
{
  public:
    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cCollisionRecorder
    cInteractionRecorder() { clear(); }

    //! Destructor of cCollisionRecorder
    virtual ~cInteractionRecorder() {};

    // METHODS:
    //! Clear all records.
    void clear()
    {
        m_interactions.clear();
    }

    // MEMBERS:
    //! List of interactions.
    vector<cInteractionEvent> m_interactions;
};


//===========================================================================
/*!
      \class    cInteractionSettings
      \brief    This structure contains a list of settings which are passed
                to the interaction detector when checking for an interaction.
*/
//===========================================================================
struct cInteractionSettings
{
    //! If true, then collision detector shall check for collisions on visible objects only.
    bool m_checkVisibleObjectsOnly;

    //! If true, then collision detector shall check for collisions on haptic enabled objects only.
    bool m_checkHapticObjectsOnly;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

