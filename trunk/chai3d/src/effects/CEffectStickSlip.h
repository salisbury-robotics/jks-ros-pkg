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
#ifndef CEffectStickSlipH
#define CEffectStickSlipH
//---------------------------------------------------------------------------
#include "effects/CGenericEffect.h"
//---------------------------------------------------------------------------

struct cStickSlipStatus
{
    // current stick position
    cVector3d m_currentStickPosition;

    // is the stick position valid?
    bool m_valid;
};

//===========================================================================
/*!
      \file     CEffectStickSlip.h
      \class    cEffectStickSlip
      \brief    cEffectStickSlip models a stick and slip effect.
*/
//===========================================================================
class cEffectStickSlip : public cGenericEffect
{
  public:
    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cEffectStickSlip.
    cEffectStickSlip(cGenericObject* a_parent);

    //! Destructor of cEffectStickSlip.
    virtual ~cEffectStickSlip() {};

    // METHODS:
    //! Compute resulting force
    bool computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_reactionForce);

  protected:

  //! store the algorythm history for each IDN calling this effect
  cStickSlipStatus m_history[CHAI_EFFECT_MAX_IDN];

};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
