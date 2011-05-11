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
#ifndef CHapticDeviceHandlerH
#define CHapticDeviceHandlerH
//---------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
#include "devices/CVirtualDevice.h"
#include "devices/CDeltaDevices.h"
#include "devices/CFalconDevice.h"
#include "devices/CFreedom6SDevice.h"
#include "devices/CPhantomDevices.h"
#include "devices/CMyCustomDevice.h"
//---------------------------------------------------------------------------
const unsigned int CHAI_MAX_HAPTIC_DEVICES = 16;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file CHapticDeviceHandler.h
    \class cHapticDeviceHandler
    \brief
    This class implements a manager which lists the different devices
    available on your computer and provides handles to them.
*/
//===========================================================================
class cHapticDeviceHandler
{
  public:
    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cHapticDeviceHandler.
    cHapticDeviceHandler();

    //! Destructor of cHapticDeviceHandler.
    virtual ~cHapticDeviceHandler();

    //! Returns the number of devices connected to your computer.
    unsigned int getNumDevices() { return (m_numDevices); }

    //! Updates information regarding the devices that are connected to your computer.
    void update();

    //! Returns the specifications of the ith device.
    int getDeviceSpecifications(cHapticDeviceInfo& a_deviceSpecifications, unsigned int a_index = 0);

    //! Returns a handle to the ith device if available.
    int getDevice(cGenericHapticDevice*& a_hapticDevice, unsigned int a_index = 0);

  private:

    //! Number of devices.
    unsigned int m_numDevices;

    //! Array of available haptic devices.
    cGenericHapticDevice* m_devices[CHAI_MAX_HAPTIC_DEVICES];

    //! A default device with no functionalities
    cGenericHapticDevice* m_nullHapticDevice;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
