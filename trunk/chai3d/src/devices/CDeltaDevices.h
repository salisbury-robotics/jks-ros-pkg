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
    \author:    Force Dimension - www.forcedimension.com
    \version    #CHAI_VERSION#
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDeltaDevicesH
#define CDeltaDevicesH
//---------------------------------------------------------------------------
#ifndef _DISABLE_DELTA_DEVICE_SUPPORT
//---------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//---------------------------------------------------------------------------

#ifdef _WIN32

/* devices */
#define DHD_DEVICE_NONE            0
#define DHD_DEVICE_SIMULATOR      11
#define DHD_DEVICE_3DOF           31
#define DHD_DEVICE_6DOF           61
#define DHD_DEVICE_6DOF_500       62
#define DHD_DEVICE_OMEGA          32
#define DHD_DEVICE_OMEGA3         33
#define DHD_DEVICE_OMEGA33        34
#define DHD_DEVICE_OMEGA33_LEFT   36
#define DHD_DEVICE_OMEGA331       35
#define DHD_DEVICE_OMEGA331_LEFT  37
#define DHD_DEVICE_CONTROLLER     81
#define DHD_DEVICE_CONTROLLER_HR  82
#define DHD_DEVICE_CUSTOM         91

/* status */
#define DHD_ON                     1
#define DHD_OFF                    0

/* device count */
#define DHD_MAX_DEVICE             4

/* TimeGuard return value */
#define DHD_TIMEGUARD              1

/* status count */
#define DHD_MAX_STATUS            13

/* status codes */
#define DHD_STATUS_POWER           0
#define DHD_STATUS_CONNECTED       1
#define DHD_STATUS_STARTED         2
#define DHD_STATUS_RESET           3
#define DHD_STATUS_IDLE            4
#define DHD_STATUS_FORCE           5
#define DHD_STATUS_BRAKE           6
#define DHD_STATUS_TORQUE          7
#define DHD_STATUS_WRIST_DETECTED  8
#define DHD_STATUS_ERROR           9
#define DHD_STATUS_GRAVITY        10
#define DHD_STATUS_TIMEGUARD      11
#define DHD_STATUS_ROTATOR_RESET  12

/* buttons count */
#define DHD_MAX_BUTTONS            8

/* velocity estimator computation mode */
#define DHD_VELOCITY_WINDOWING     0
#define DHD_VELOCITY_AVERAGING     1

#endif

//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file   CDeltaDevices.h
    \class  cDeltaDevice
    \brief  cDeltaDevice describes an interface to the Delta and Omega haptic
			devices from Force Dimension.
*/
//===========================================================================
class cDeltaDevice : public cGenericHapticDevice
{
  public:

    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cDeltaDevices.
    cDeltaDevice(unsigned int a_deviceNumber = 0);

    //! Destructor of cGenericDevice.
    virtual ~cDeltaDevice();

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

    //! Read the gripper angle in radian
    int getGripperAngleRad(double& a_angle);

    //! Send a force [N] to the haptic device
    int setForce(cVector3d& a_force);

    //! Send a torque [N*m] to the haptic device
    int setTorque(cVector3d& a_torque);

    //! Send a torque [N*m] to the gripper
    int setGripperTorque(double a_gripperTorque);

    //! Send a force [N] and a torque [N*m] and gripper torque [N*m] to the haptic device.
    int setForceAndTorqueAndGripper(cVector3d& a_force, cVector3d& a_torque, double a_gripperTorque);

    //! read the status of the user switch [1 = ON / 0 = OFF]
    int getUserSwitch(int a_switchIndex, int& a_status);

    //-----------------------------------------------------------------------
    // METHODS RESTRICTED TO FORCE DIMENSION DEVICES ONLY:
    //-----------------------------------------------------------------------

    //! Return the Force Dimension type of the current device.
    int getDeviceType() { return m_deviceType; }

    //! Overrides the force button switch located at the base of the device
    int enableForces(bool a_value);

  protected:

    //! Reference count used to control access to the dhd dll
    static int m_activeDeltaDevices;

    //! Device ID number
    int m_deviceID;

    //! Which FD device is actually instantiated here?
    int m_deviceType;

    //! Last position of user switch
    int m_userSwitchCount[8];
	int m_userSwitchStatus[8];
    cPrecisionClock m_userSwitchClock[8];

    //! Read user switch from end-effector
    int getUserSwitch(int a_deviceID);

    //! Have forces been enable yet since the connexion to the device was opened?
    bool statusEnableForcesFirstTime;
};

//---------------------------------------------------------------------------
#endif //_DISABLE_DELTA_DEVICE_SUPPORT
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
