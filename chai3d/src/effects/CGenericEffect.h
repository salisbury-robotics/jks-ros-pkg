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
#ifndef CGenericEffectH
#define CGenericEffectH
//---------------------------------------------------------------------------
#include "extras/CGlobals.h"
#include "math/CVector3d.h"
//---------------------------------------------------------------------------
class cGenericObject;
//---------------------------------------------------------------------------
const int CHAI_EFFECT_MAX_IDN = 16;
//---------------------------------------------------------------------------

//===========================================================================
/*!
      \file     CGenericEffect.h
      \class    cGenericEffect
      \brief    cGenericEffect provides a base class to program haptic
      effects (force models) when a virtual tool interacts with objects
      of a virtual environment.
*/
//===========================================================================
class cGenericEffect
{
  public:
    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of CGenericEffect.
    cGenericEffect(cGenericObject* a_parent); 

    //! Destructor of CGenericEffect.
    virtual ~cGenericEffect() {};

    // METHODS:
    //! Compute a resulting force
    virtual bool computeForce(const cVector3d& a_toolPos,
                              const cVector3d& a_toolVel,
                              const unsigned int& a_toolID,
                              cVector3d& a_reactionForce)
                              {
                                  a_reactionForce.zero();
                                  return (false);
                              }

    //! Read last computed force
    cVector3d getLastComputedForce() { return (m_lastComputedForce); }

    //! object to which the force effects applies
    cGenericObject* m_parent;

    //! Enable or disable current effect
    inline void enable(bool a_status) { m_enabled = a_status; }

    //! Is the current effect enabled
    inline bool isEnabled() { return (m_enabled); }

  protected:

    //! last computed force
    cVector3d m_lastComputedForce;

    //! initialize effect model.
    virtual void initialize() { return; }

    //! is the current effect enabled
    bool m_enabled;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
