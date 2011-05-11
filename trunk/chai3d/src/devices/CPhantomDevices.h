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
    \author:    Federico Barbagli
    \author:    Francois Conti
    \version    #CHAI_VERSION#
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CPhantomDevicesH
#define CPhantomDevicesH
//---------------------------------------------------------------------------
#ifndef _DISABLE_PHANTOM_DEVICE_SUPPORT
//---------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//---------------------------------------------------------------------------
#ifdef _LINUX
#include "devices/hdPhantom.h"
#endif
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \class  cPhantomDevice
    \brief  cPhantomDevice describes an interface to the PHANToM haptic
			devices from Sensable Technologies
*/
//===========================================================================
class cPhantomDevice : public cGenericHapticDevice
{
  public:
    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cPhantomDevice.
    cPhantomDevice(unsigned int a_deviceNumber);

    //! Destructor of cPhantomDevices.
    ~cPhantomDevice();

    // METHODS:
    //! Open connection to haptic device (0 indicates success)
    int open();

    //! Close connection to haptic device (0 indicates success)
    int close();

    //! Initialize or calibrate haptic device (0 indicates success)
    int initialize(const bool a_resetEncoders=false);

    //! Returns the number of devices available from this class of device.
    unsigned int getNumDevices();

    //! Read the position of the device. Units are in meters.
    int getPosition(cVector3d& a_position);

    //! Read the linear velocity of the device
    int getLinearVelocity(cVector3d& a_linearVelocity);

    //! Read the orientation frame of the device end-effector
    int getRotation(cMatrix3d& a_rotation);

    //! Send a force [N] to the haptic device
    int setForce(cVector3d& a_force);

    //! Send a torque [N*m] to the haptic device
    int setTorque(cVector3d& a_torque);

    //! read the status of the user switch [1 = ON / 0 = OFF]
    int getUserSwitch(int a_switchIndex, int& a_status);


  private:
    //! counter
    static int m_dllcount;

    //! Device ID number
    int m_deviceID;

    //! Are the Falcon drivers installed and available
    bool m_driverInstalled;
};

//---------------------------------------------------------------------------
#endif // #ifndef _DISABLE_PHANTOM_SUPPORT
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

