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
    \author:    Federico Barbagli
    \version    #CHAI_VERSION#
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "tools/CGenericTool.h"
//---------------------------------------------------------------------------

//==========================================================================
/*!
      Constructor of cGenericTool.

      \fn       cGenericTool::cGenericTool()
*/
//===========================================================================
cGenericTool::cGenericTool()
{
    // no device is currently connected to this tool
    m_device = NULL;

    // initialize variable which stores the status of the user switches of a
    // the device
    m_userSwitches = 0;

}


//==========================================================================
/*!
      Read the status of the user switches from the haptic device
      controlled by this tool

      \fn       int cGenericTool::getUserSwitch(int a_switchIndex)
      \param    a_switchIndex Index number of the switch
      \return   Return 1 if switch is active, otherwise return 0.
*/
//===========================================================================
int cGenericTool::getUserSwitch(int a_switchIndex)
{
    int result = 0;

    switch (a_switchIndex)
    {
        case 0: result = (m_userSwitches & 1); break;
        case 1: result = (m_userSwitches & 2); break;
        case 2: result = (m_userSwitches & 4); break;
        case 3: result = (m_userSwitches & 8); break;
    }

    return (result);
}
