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
    \author    Stephen Sinclair
    \author    http://www.mpb-technologies.ca/
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 699 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CFreedom6SDeviceH
#define CFreedom6SDeviceH
//---------------------------------------------------------------------------
#if defined(_ENABLE_MPB_DEVICE_SUPPORT)
//---------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CFreedom6SDevice.h

    \brief
    <b> Devices </b> \n 
    Freedom 6S Haptic Device.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cFreedom6SDevice
    \ingroup    devices  

    \brief  
    cFreedom6SDevice describes an interface to the Freedom6S haptic
    device from MPB Technologies Inc.
*/
//===========================================================================
class cFreedom6SDevice : public cGenericHapticDevice
{
  public:
 
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cFreedom6SDevice.
    cFreedom6SDevice();

    //! Destructor of cFreedom6SDevice.
    virtual ~cFreedom6SDevice();


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Open connection to MPB device.
    int open();

    //! Close connection to MPB device.
    int close();

    //! Calibrate MPB device.
    int calibrate();

    //! Returns the number of devices available from this class of device.
    unsigned int getNumDevices();

    //! Read the position of the device. Units are meters [m].
    int getPosition(cVector3d& a_position);

    //! Read the linear velocity of the device. Units are in meters per second [m/s].
    int getLinearVelocity(cVector3d& a_linearVelocity);

    //! Read the orientation frame of the device end-effector.
    int getRotation(cMatrix3d& a_rotation);

    //! Read the angular velocity of the device. Units are in radians per second [m/s].
    int getAngularVelocity(cVector3d& a_angularVelocity);

    //! Read the gripper angle in radian.
    int getGripperAngleRAD(double& a_angle);

    //! Send a force [N] to the haptic device.
    int setForce(cVector3d& a_force);

    //! Send a force [N] and a torque [N*m] to the haptic device.
    int setForceAndTorque(cVector3d& a_force, cVector3d& a_torque);

    //! Read the status of the user switch [\b true = \b ON / \b false = \b OFF].
    int getUserSwitch(int a_switchIndex, bool& a_status);


  protected:

    //! Reference count used to control access to the DLL.
    static int m_activeFreedom6SDevices;

    //! Handle to device.
    void* m_hf6s;
};

//---------------------------------------------------------------------------
#endif  // _ENABLE_MPB_DEVICE_SUPPORT
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

