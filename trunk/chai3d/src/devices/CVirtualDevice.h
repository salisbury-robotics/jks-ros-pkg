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
#ifndef CVirtualDeviceH
#define CVirtualDeviceH
//---------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//---------------------------------------------------------------------------
#ifndef _DISABLE_VIRTUAL_DEVICE_SUPPORT
//---------------------------------------------------------------------------

struct cVirtualDeviceData
{
    double       ForceX;   // Force component X.
    double       ForceY;   // Force component Y.
    double       ForceZ;   // Force component Z.
    double       TorqueA;  // Torque alpha.
    double       TorqueB;  // Torque beta.
    double       TorqueG;  // Torque gamma.
    double       PosX;     // Position X.
    double       PosY;     // Position Y.
    double       PosZ;     // Position Z.
    double       AngleA;   // Angle alpha.
    double       AngleB;   // Angle beta.
    double       AngleG;   // Angle gamma.
    bool         Button0;  // Button 0 status.
    bool         AckMsg;   // Acknowledge Message
    bool         CmdReset; // Command Reset
};

//===========================================================================
/*!
    \file CVirtualDevice.h
    \class cVirtualDevice
    \brief
    Class which interfaces with the virtual device
*/
//===========================================================================
class cVirtualDevice : public cGenericHapticDevice
{
  public:
    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cVirtualDevices.
    cVirtualDevice();

    //! Destructor of cGenericDevice.
    virtual ~cVirtualDevice();

    // METHODS:
    //! Open connection to virtual device.
    int open();

    //! Close connection to virtual device
    int close();

    //! Initialize virtual device
    int initialize(const bool a_resetEncoders);

    //! Returns the number of devices available from this class of device.
    unsigned int getNumDevices();

    //! Read the position of the device. Units are in meters.
    int getPosition(cVector3d& a_position);

    //! Read the orientation frame of the device end-effector
    int getRotation(cMatrix3d& a_rotation);

    //! read the status of the user switch [1 = ON / 0 = OFF]
    int getUserSwitch(int a_switchIndex, int& a_status);

    //! Send a force [N] to the haptic device
    int setForce(cVector3d& a_force);

    //! Read a force [N] from the haptic device.
    int getForce(cVector3d& a_force);

  private:
    //! shared memory connection to virtual haptic device
    HANDLE m_hMapFile;

    //! pointer to shared memory
    LPVOID m_lpMapAddress;

    //! pointer to shared memory data structure
    cVirtualDeviceData* m_pDevice;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
#endif // _DISABLE_VIRTUAL_DEVICE_SUPPORT
//---------------------------------------------------------------------------

