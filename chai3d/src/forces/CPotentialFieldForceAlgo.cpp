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
#include "forces/CInteractionBasics.h"
#include "forces/CPotentialFieldForceAlgo.h"
#include "scenegraph/CWorld.h"
//---------------------------------------------------------------------------
unsigned int cPotentialFieldForceAlgo::m_IDNcounter = 0;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Coonstructor of cPotentialFieldForceAlgo

    \fn       cPotentialFieldForceAlgo::cPotentialFieldForceAlgo()
*/
//===========================================================================
cPotentialFieldForceAlgo::cPotentialFieldForceAlgo()
{
    // define an identification number for this force algorythm
    m_IDN = m_IDNcounter;

    // increment counter
    m_IDNcounter++;

    // define default settings
    m_interactionSettings.m_checkVisibleObjectsOnly = true;
    m_interactionSettings.m_checkHapticObjectsOnly  = true;
}


//===========================================================================
/*!
    Compute forces for all potential field based objects (cGenericPotentialField).

    \fn       cVector3d cPotentialFieldForceAlgo::computeForces(const
              cVector3d& a_nextDevicePos)
    \param    a_nextDevicePos  Next position of haptic device or virtual finger.
*/
//===========================================================================
cVector3d cPotentialFieldForceAlgo::computeForces(const cVector3d& a_toolPos,
                                                  const cVector3d& a_toolVel)
{
    // initialize force
    cVector3d force;
    force.zero();
    m_interactionRecorder.m_interactions.clear();

    // compute force feedback for all potential field based objects located
    // in the world
    if (m_world != NULL)
    {
        force = m_world->computeInteractions(a_toolPos,
                                             a_toolVel,
                                             m_IDN,
                                             m_interactionRecorder,
                                             m_interactionSettings);
    }

    // return result
    return (force);
}

