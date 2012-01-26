//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2012, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 721 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "extras/CGlobals.h"
#include "devices/CVirtualDevice.h"
//---------------------------------------------------------------------------
#if defined(_ENABLE_VIRTUAL_DEVICE_SUPPORT)
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cVirtualDevice.

    \fn     cVirtualDevice::cVirtualDevice()
*/
//===========================================================================
cVirtualDevice::cVirtualDevice()
{
    // settings:
    m_specifications.m_manufacturerName              = "CHAI3D";
    m_specifications.m_modelName                     = "virtual";
    m_specifications.m_maxLinearForce                = 10.0;    // [N]
    m_specifications.m_maxAngularTorque              = 0.0;     // [N*m]
    m_specifications.m_maxGripperForce               = 0.0;     // [N]
    m_specifications.m_maxLinearStiffness            = 1000.0;  // [N/m]
    m_specifications.m_maxAngularStiffness           = 0.0;     // [N*m/Rad]
    m_specifications.m_maxGripperLinearStiffness     = 0.0;     // [N/m]
    m_specifications.m_maxLinearDamping              = 100;     // [N/(m/s)]
    m_specifications.m_maxAngularDamping			 = 0.0;     // [N*m/(Rad/s)]
    m_specifications.m_maxGripperAngularDamping	     = 0.0;     // [N*m/(Rad/s)]
    m_specifications.m_workspaceRadius               = 0.15;    // [m]
    m_specifications.m_sensedPosition                = true;
    m_specifications.m_sensedRotation                = false;
    m_specifications.m_sensedGripper                 = false;
    m_specifications.m_actuatedPosition              = true;
    m_specifications.m_actuatedRotation              = false;
    m_specifications.m_actuatedGripper               = false;
    m_specifications.m_leftHand                      = true;
    m_specifications.m_rightHand                     = true;

    m_systemAvailable = false;
    m_systemReady = false;

    // search for virtual device
    m_hMapFile = OpenFileMapping(
        FILE_MAP_ALL_ACCESS,
        FALSE,
        "dhdVirtual");

    // no virtual device available
    if (m_hMapFile == NULL)
    {
        m_systemReady = false;
        m_systemAvailable = false;
        return;
    }

    // open connection to virtual device
    m_lpMapAddress = MapViewOfFile(
      m_hMapFile,
      FILE_MAP_ALL_ACCESS,
      0,
      0,
      0);

    // check whether connection succeeded
    if (m_lpMapAddress == NULL)
    {
        m_systemReady = false;
        m_systemAvailable = false;
        return;
    }

    // map memory
    m_pDevice = (cVirtualDeviceData*)m_lpMapAddress;

    // virtual device is available
    m_systemAvailable = true;
}


//===========================================================================
/*!
    Destructor of cVirtualDevice.

    \fn         cVirtualDevice::~cVirtualDevice()
*/
//===========================================================================
cVirtualDevice::~cVirtualDevice()
{
    if (m_systemAvailable)
    {
        CloseHandle(m_hMapFile);
    }
}


//===========================================================================
/*!
    Open connection to virtual device.

    \fn     int cVirtualDevice::open()
    \return Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cVirtualDevice::open()
{
    if (m_systemAvailable)
    {
        m_systemReady = true;
    }
    return (0);
}


//===========================================================================
/*!
    Close connection to virtual device

    \fn     int cVirtualDevice::close()
    \return Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cVirtualDevice::close()
{
    m_systemReady = false;

    return (0);
}


//===========================================================================
/*!
    Calibrate virtual device.  

    \fn     void cVirtualDevice::calibrate()
    \return Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cVirtualDevice::calibrate()
{
    if (m_systemReady)
    {
        return (0);
    }
    else
    {
        return (-1);
    }
}


//===========================================================================
/*!
    Returns the number of devices available from this class of device.

    \fn      unsigned int cVirtualDevice::getNumDevices()
    \return  Returns the result
*/
//===========================================================================
unsigned int cVirtualDevice::getNumDevices()
{
    // only one device can be enabled
    int result;
    if (m_systemAvailable)
    {
        result = 1;
    }
    else
    {
        result = 0;
    }
    return (result);
}


//===========================================================================
/*!
    Read the position of the device. Units are meters [m].

    \fn     int cVirtualDevice::getPosition(cVector3d& a_position)
    \param  a_position  Return value.
*/
//===========================================================================
int cVirtualDevice::getPosition(cVector3d& a_position)
{
    if (!m_systemReady)
    {
        a_position.set(0, 0, 0);
        return (-1);
    }

    double x,y,z;
    x = (double)(*m_pDevice).PosX;
    y = (double)(*m_pDevice).PosY;
    z = (double)(*m_pDevice).PosZ;
    a_position.set(x, y, z);

    return (0);
}


//===========================================================================
/*!
    Read the orientation frame of the device end-effector.

    \fn     int cVirtualDevice::getRotation(cMatrix3d& a_rotation)
    \param  a_rotation  Return value.
*/
//===========================================================================
int cVirtualDevice::getRotation(cMatrix3d& a_rotation)
{
    if (!m_systemReady)
    {
        a_rotation.identity();
        return (-1);
    }

    a_rotation.identity();

    return (0);
}


//===========================================================================
/*!
    Send a force [N] to the haptic device.

    \fn     int cVirtualDevice::setForce(cVector3d& a_force)
    \param  a_force  Force command to be applied to device.
*/
//===========================================================================
int cVirtualDevice::setForce(cVector3d& a_force)
{
    if (!m_systemReady) return (-1);

    ((*m_pDevice).ForceX) = a_force(0) ;
    ((*m_pDevice).ForceY) = a_force(1) ;
    ((*m_pDevice).ForceZ) = a_force(2) ;

    return (0);
}


//===========================================================================
/*!
    Return the last force sent to the device.

    \fn     int cVirtualDevice::getForce(cVector3d& a_force)
    \param  a_force  Return value.
*/
//===========================================================================
int cVirtualDevice::getForce(cVector3d& a_force)
{
    if (!m_systemReady)
    {
        a_force.set(0,0,0);
        return (-1);
    }

    a_force(0)  = ((*m_pDevice).ForceX);
    a_force(1)  = ((*m_pDevice).ForceY);
    a_force(2)  = ((*m_pDevice).ForceZ);

    return (0);
}

//===========================================================================
/*!
    Read the status of the user switch [\b true = \e ON / \b false = \e OFF].

    \fn     int cVirtualDevice::getUserSwitch(int a_switchIndex, bool& a_status)
    \param  a_switchIndex  index number of the switch.
    \param  a_status result value from reading the selected input switch.
*/
//===========================================================================
int cVirtualDevice::getUserSwitch(int a_switchIndex, bool& a_status)
{
    if (!m_systemReady)
    {
        a_status = false;
        return (-1);
    }

    a_status = ((bool)(*m_pDevice).Button0);

    return (0);
}


//---------------------------------------------------------------------------
#endif  // _ENABLE_VIRTUAL_DEVICE_SUPPORT
//---------------------------------------------------------------------------


