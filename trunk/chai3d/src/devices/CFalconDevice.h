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
#ifndef CFalconDeviceH
#define CFalconDeviceH
//---------------------------------------------------------------------------
#ifndef _DISABLE_FALCON_DEVICE_SUPPORT
//---------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \class  cFalconDevice
    \brief  cFalconDevice describes an interface to the Delta and Omega haptic
			devices from Force Dimension.
*/
//===========================================================================
class cFalconDevice : public cGenericHapticDevice
{
  public:

    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cFalconDevices.
    cFalconDevice(unsigned int a_deviceNumber = 0);

    //! Destructor of cGenericDevice.
    virtual ~cFalconDevice();

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

    //! Send a force [N] to the haptic device
    int setForce(cVector3d& a_force);

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
#endif
//---------------------------------------------------------------------------
#endif //_DISABLE_FALCON_DEVICE_SUPPORT
//---------------------------------------------------------------------------
