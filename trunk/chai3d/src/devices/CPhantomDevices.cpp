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
#include "devices/CPhantomDevices.h"
//---------------------------------------------------------------------------
#ifndef _DISABLE_PHANTOM_DEVICE_SUPPORT
//---------------------------------------------------------------------------
#if defined(_WIN32)
HINSTANCE hdPhantomDLL = NULL;
HINSTANCE hdPhantomDriverDLL = NULL;

int (__stdcall *hdPhantomGetNumDevices)  ();
int (__stdcall *hdPhantomOpen)           (int a_deviceID);
int (__stdcall *hdPhantomClose)          (int a_deviceID);
int (__stdcall *hdPhantomGetPosition)    (int a_deviceID,
                                          double *a_posX,
                                          double *a_posY,
                                          double *a_posZ);
int (__stdcall *hdPhantomGetLinearVelocity)(int a_deviceID,
                                            double *a_velX,
                                            double *a_velY,
                                            double *a_velZ);
int (__stdcall *hdPhantomGetRotation)    (int a_deviceID,
                                          double *a_rot00,
                                          double *a_rot01,
                                          double *a_rot02,
                                          double *a_rot10,
                                          double *a_rot11,
                                          double *a_rot12,
                                          double *a_rot20,
                                          double *a_rot21,
                                          double *a_rot22);
int (__stdcall *hdPhantomGetButtons)     (int a_deviceID);
int (__stdcall *hdPhantomSetForce)       (int a_deviceID,
                                          double *a_forceX,
			                              double *a_forceY,
			                              double *a_forceZ);
int (__stdcall *hdPhantomSetTorque)      (int a_deviceID,
                                          double *a_torqueX,
			                              double *a_torqueY,
			                              double *a_torqueZ);
int (__stdcall *hdPhantomGetWorkspaceRadius)(int a_deviceID,
							                 double *a_workspaceRadius);
int (__stdcall *hdPhantomGetType)(int a_deviceID,
                                  char* a_typeName);


// Initialize dhd dll reference count
int cPhantomDevice::m_dllcount = 0;
#endif

//===========================================================================
/*!
    Constructor of cPhantomDevice.
    No servo loop is yet created, encoders are NOT reset.

    \fn     cPhantomDevice::cPhantomDevice(int a_num, bool a_dio_access)
    \param  a_deviceNumber  Index number to the ith Phantom device
*/
//===========================================================================
cPhantomDevice::cPhantomDevice(unsigned int a_deviceNumber)
{
    // default specification setup
    m_specifications.m_manufacturerName              = "Sensable Technologies";
    m_specifications.m_modelName                     = "PHANTOM";
    m_specifications.m_maxForce                      = 6.0;     // [N]
    m_specifications.m_maxForceStiffness             = 1000.0;  // [N/m]
    m_specifications.m_maxTorque                     = 0.0;     // [N*m]
    m_specifications.m_maxTorqueStiffness            = 0.0;     // [N*m/Rad]
    m_specifications.m_maxGripperTorque              = 0.0;     // [N]
    m_specifications.m_maxGripperTorqueStiffness     = 0.0;     // [N*m/m]
    m_specifications.m_workspaceRadius               = 0.10;    // [m];
    m_specifications.m_sensedPosition                = true;
    m_specifications.m_sensedRotation                = true;
    m_specifications.m_sensedGripper                 = false;
    m_specifications.m_actuatedPosition              = true;
    m_specifications.m_actuatedRotation              = false;
    m_specifications.m_actuatedGripper               = false;
    m_specifications.m_leftHand                      = true;
    m_specifications.m_rightHand                     = true;

    // device is not yet available or ready
    m_driverInstalled = false;
    m_systemAvailable = false;
    m_systemReady = false;

    // check if Phantom drivers installed
    if (m_dllcount == 0)
    {
        hdPhantomDriverDLL = LoadLibrary("HD.dll");
        if (hdPhantomDriverDLL == NULL) { return; }
    }

    // the Phantom drivers are installed
    m_driverInstalled = true;

    // load dll library
    if (m_dllcount == 0)
    {
        hdPhantomDLL = LoadLibrary("hdPhantom.dll");
    }

    // check if DLL loaded correctly
    if (hdPhantomDLL == NULL)
    {
        return;
    }

    // load different callbacks
    hdPhantomGetNumDevices = (int (__stdcall*)(void))
                            GetProcAddress(hdPhantomDLL, "hdPhantomGetNumDevices");

    hdPhantomOpen         = (int (__stdcall*)(int))
                            GetProcAddress(hdPhantomDLL, "hdPhantomOpen");

    hdPhantomClose        = (int (__stdcall*)(int))
                            GetProcAddress(hdPhantomDLL, "hdPhantomClose");

    hdPhantomGetPosition  = (int (__stdcall*)(int,
                                              double*, double*, double*))
                            GetProcAddress(hdPhantomDLL, "hdPhantomGetPosition");

    hdPhantomGetLinearVelocity  = (int (__stdcall*)(int,
                                                    double*, double*, double*))
                            GetProcAddress(hdPhantomDLL, "hdPhantomGetLinearVelocity");

    hdPhantomGetRotation  = (int (__stdcall*)(int,
                                              double*, double*, double*,
                                              double*, double*, double*,
                                              double*, double*, double*))
                            GetProcAddress(hdPhantomDLL, "hdPhantomGetRotation");

    hdPhantomGetButtons   = (int (__stdcall*)(int))
                            GetProcAddress(hdPhantomDLL, "hdPhantomGetButtons");

    hdPhantomSetForce     = (int (__stdcall*)(int,
                                              double*,
                                              double*,
                                              double*))
                            GetProcAddress(hdPhantomDLL, "hdPhantomSetForce");
    hdPhantomSetTorque    = (int (__stdcall*)(int,
                                              double*,
                                              double*,
                                              double*))
                            GetProcAddress(hdPhantomDLL, "hdPhantomSetTorque");
    hdPhantomGetWorkspaceRadius = (int (__stdcall*)(int,
                                                    double*))
                            GetProcAddress(hdPhantomDLL, "hdPhantomGetWorkspaceRadius");

    hdPhantomGetType      = (int (__stdcall*)(int,
                                              char*))
                            GetProcAddress(hdPhantomDLL, "hdPhantomGetType");


    // get the number ID of the device we wish to communicate with
    m_deviceID = a_deviceNumber;

    // get the number of Force Dimension devices connected to this computer
    int numDevices = hdPhantomGetNumDevices();

    // check if such device is available
    if ((a_deviceNumber + 1) > (unsigned int)numDevices)
    {
        // no, such ID does not lead to an existing device
        m_systemAvailable = false;
    }
    else
    {
        // yes, this ID leads to an existing device
        m_systemAvailable = true;
    }

    // read information related to the device
    hdPhantomGetWorkspaceRadius(m_deviceID, &m_specifications.m_workspaceRadius);

    // read the device model
    char name[255];
    hdPhantomGetType(m_deviceID, &name[0]);
    m_specifications.m_modelName = name;

    /////////////////////////////////////////////////////////////////////
    // Define specifications given the device model
    /////////////////////////////////////////////////////////////////////

    if (m_specifications.m_modelName == "PHANTOM Omni")
    {
        m_specifications.m_maxForce                      = 4.0;     // [N]
        m_specifications.m_maxForceStiffness             = 700.0;   // [N/m]
    }

    // increment counter
    m_dllcount++;
}


//===========================================================================
/*!
    Destructor of cPhantomDevice.

    \fn     cPhantomDevice::~cPhantomDevice()
*/
//===========================================================================
cPhantomDevice::~cPhantomDevice()
{
    // close connection to device
    if (m_systemReady)
    {
        close();
    }

    m_dllcount--;

    if ((m_dllcount == 0) && (hdPhantomDLL != NULL))
    {
        FreeLibrary(hdPhantomDLL);
        hdPhantomDLL = NULL;
    }
}


//===========================================================================
/*!
    Open connection to phantom device.

    \fn     int cPhantomDevice::open()
    \return Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cPhantomDevice::open()
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    // check if the system is available
    if (!m_systemAvailable) return (-1);

    // if system is already opened then return
    if (m_systemReady) return (0);

    // try to open the device
    hdPhantomOpen(m_deviceID);

    // update device status
    m_systemReady = true;

    // success
    return (0);
}


//===========================================================================
/*!
    Close connection to phantom device.

    \fn     int cPhantomDevice::close()
    \return Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cPhantomDevice::close()
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    // check if the system has been opened previously
    if (!m_systemReady) return (-1);

    // yes, the device is open so let's close it
    int result = hdPhantomClose(m_deviceID);

    // update status
    m_systemReady = false;

    // exit
    return (result);
}


//===========================================================================
/*!
    Initialize the phantom device.
    
    For desktops and omnis, the a_resetEncoders parameter is ignored.
    For premiums, if you specify a_resetEncoders as true, you should
    be holding the Phantom in its rest position when this is called.
    
    \fn     int cPhantomDevice::initialize(const bool a_resetEncoders=false)
    \param  a_resetEncoders Should I re-zero the encoders?  (affects premiums only...)
    \return Return 0 if operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cPhantomDevice::initialize(const bool a_resetEncoders)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    // exit
    return 0;
}


//===========================================================================
/*!
    Returns the number of devices available from this class of device.

    \fn      unsigned int cPhantomDevice::getNumDevices()
    \return  Returns the result
*/
//===========================================================================
unsigned int cPhantomDevice::getNumDevices()
{
    // check if drivers are installed
    if (!m_driverInstalled) return (0);

    // read number of devices
    int numDevices = hdPhantomGetNumDevices();
    return (numDevices);
}


//===========================================================================
/*!
    Read the position of the device. Units are in meters.

    \fn     int cPhantomDevice::getPosition(cVector3d& a_position)
    \param  a_position  Return value.
    \return Return 0 if no error occured.
*/
//===========================================================================
int cPhantomDevice::getPosition(cVector3d& a_position)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    double x,y,z;
    int error = hdPhantomGetPosition(m_deviceID, &x, &y, &z);
    a_position.set(x, y, z);
    estimateLinearVelocity(a_position);
    return (error);
}


//===========================================================================
/*!
    Read the linear velocity of the device. Units are in [m/s].

    \fn     int cPhantomDevice::getLinearVelocity(cVector3d& a_linearVelocity)
    \param  a_linearVelocity  Return value.
    \return Return 0 if no error occured.
*/
//===========================================================================
int cPhantomDevice::getLinearVelocity(cVector3d& a_linearVelocity)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    int error = -1;
    /*
    int error = -1;
    double vx,vy,vz;
    error = hdPhantomGetLinearVelocity(m_deviceID, &vx, &vy, &vz);

    m_linearVelocity.set(vx, vy, vz);
    a_linearVelocity = m_linearVelocity;
    */

    a_linearVelocity = m_linearVelocity;

    return (error);
}


//===========================================================================
/*!
    Read the orientation frame of the device end-effector

    \fn     int cPhantomDevice::getRotation(cMatrix3d& a_rotation)
    \param  a_rotation  Return value.
    \return Return 0 if no error occured.
*/
//===========================================================================
int cPhantomDevice::getRotation(cMatrix3d& a_rotation)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    double rot[3][3];
    int error = hdPhantomGetRotation(m_deviceID,
                                     &rot[0][0],
                                     &rot[0][1],
                                     &rot[0][2],
                                     &rot[1][0],
                                     &rot[1][1],
                                     &rot[1][2],
                                     &rot[2][0],
                                     &rot[2][1],
                                     &rot[2][2]);
    a_rotation.set(rot[0][0], rot[0][1], rot[0][2],
                   rot[1][0], rot[1][1], rot[1][2],
                   rot[2][0], rot[2][1], rot[2][2]);
                   
    estimateAngularVelocity(a_rotation);
    return (error);
}


//===========================================================================
/*!
    Send a force [N] to the haptic device

    \fn     int cPhantomDevice::setForce(cVector3d& a_force)
    \param  a_force  Force command to be applied to device.
    \return Return 0 if no error occured.
*/
//===========================================================================
int cPhantomDevice::setForce(cVector3d& a_force)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    int error = hdPhantomSetForce(m_deviceID, &a_force.x, &a_force.y, &a_force.z);
    m_prevForce = a_force;
    return (error);
}


//===========================================================================
/*!
    Send a torque [N*m] to the haptic device

    \fn     int cPhantomDevice::setTorque(cVector3d& a_torque)
    \param  a_torque Force command to be applied to device.
    \return Return 0 if no error occured.
*/
//===========================================================================
int cPhantomDevice::setTorque(cVector3d& a_torque)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    int error = hdPhantomSetTorque(m_deviceID, &a_torque.x, &a_torque.y, &a_torque.z);
    m_prevTorque = a_torque;
    return (error);
}


//===========================================================================
/*!
    Read the status of the user switch [1 = ON / 0 = OFF].

    \fn     int cPhantomDevice::getUserSwitch(int a_switchIndex, int& a_status)
    \param  a_switchIndex  index number of the switch.
    \param  a_status result value from reading the selected input switch.
    \return Return 0 if no error occured.
*/
//===========================================================================
int cPhantomDevice::getUserSwitch(int a_switchIndex, int& a_status)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);
    
    int result = 0;
    int button = hdPhantomGetButtons(m_deviceID);

    switch (a_switchIndex)
    {
        case 0:
            result = button & 1;
            break;

        case 1:
            result = button & 2;
            break;

        case 2:
            result = button & 3;
            break;

        case 3:
            result = button & 4;
            break;
    }

    if (result > 0)
    {
        a_status = 1;
    }
    else
    {
        a_status = 0;
    }

    return (0);
}

//---------------------------------------------------------------------------
#endif // _DISABLE_PHANTOM_DEVICE_SUPPORT
//---------------------------------------------------------------------------

