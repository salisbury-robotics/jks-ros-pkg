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
#ifndef CGenericHapticDeviceH
#define CGenericHapticDeviceH
//---------------------------------------------------------------------------
#include "devices/CGenericDevice.h"
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
#include "timers/CPrecisionClock.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file CHapticDeviceHandler
    \struct cHapticDeviceInfo
    \brief
    Provides a structure which can hold technical specifications about a
    particular haptic device.
*/
//===========================================================================
struct cHapticDeviceInfo
{
    //! Name of the device model. "delta, "omega" or "phantom" for instance.
    string m_modelName;

    //! Name of the manufacturer of the device.
    string m_manufacturerName;

    //! Maximum force in [N] that can be produced by the device in translation.
    double m_maxForce;

    //! Maximum torque in [N*m] that can be produced by the device in orientation.
    double m_maxTorque;

    //! Maximum force in [N*m] that can be produced by the gripper.
    double m_maxGripperTorque;

    //! Maximum closed loop force stiffness [N/m] for a simulation running at 1 KhZ.
    double m_maxForceStiffness;

    //! Maximum closed loop torque stiffness [N*m/rad] for a simulation running at 1 KhZ.
    double m_maxTorqueStiffness;

    //! Maximum closed loop gripper torque stiffness [N*m/rad] for a simulation running at 1 KhZ.
    double m_maxGripperTorqueStiffness;

    //! Radius which describes the larget sphere (3D devices) or circle (2D Devices) which can be enclosed inside the physical workspace of the device.
    double m_workspaceRadius;

    //! If \b true then device supports position sensing (x,y,z axis).
    bool m_sensedPosition;

    //! If \b true thhen device supports rotation sensing. (i.e stylus, pen).
    bool m_sensedRotation;

    //! If \b true then device supports a sensed gripper interface.
    bool m_sensedGripper;

    //! If \b true then device provides actuation capabilities on the translation degrees of freedom. (x,y,z axis).
    bool m_actuatedPosition;

    //! If \b true then device provides actuation capabilities on the orientation degrees of freedom. (i.e stylus, pen).
    bool m_actuatedRotation;

    //! If \b true then device provides actuation capabilities on the gripper.
    bool m_actuatedGripper;

    //! If \b true then the device can used for left hands.
    bool m_leftHand;

    //! If \b true then the device can used for left hands.
    bool m_rightHand;
};


//===========================================================================
/*!
    \class  cGenericHapticDevice
    \brief  cGenericHapticDevice describes a virtual class from which all
            2D or 3D point contact haptic devices are derived. These include
            for instance the DELTA, OMEGA or PHANTOM haptic devices
*/
//===========================================================================
class cGenericHapticDevice : public cGenericDevice
{
  public:
    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cGenericDevice.
    cGenericHapticDevice();

    //! Destructor of cGenericDevice.
    virtual ~cGenericHapticDevice() {};


    // VIRTUAL METHODS:
    //! Open connection to haptic device (0 indicates success)
    virtual int open() { return -1; }

    //! Close connection to haptic device (0 indicates success)
    virtual int close() { return -1; }

    //! Initialize or calibrate haptic device (0 indicates success)
    virtual int initialize(const bool a_resetEncoders=false) { return -1; }

    //! Send a generic command to the haptic device (0 indicates success)
    virtual int command(int a_command, void* a_data);

    //! Read the position of the device. Units are in meters.
    virtual int getPosition(cVector3d& a_position) { a_position.zero(); return (0); }

    //! Read the linear velocity of the device. Units are in meters per second [m/s].
    virtual int getLinearVelocity(cVector3d& a_linearVelocity) { a_linearVelocity = m_linearVelocity; return (0); }

    //! Read the orientation frame of the device end-effector
    virtual int getRotation(cMatrix3d& a_rotation) { a_rotation.identity(); return (0); }

    //! Read the angular velocity of the device. Units are in radians per second [m/s].
    virtual int getAngularVelocity(cVector3d& a_angularVelocity) { a_angularVelocity = m_angularVelocity; return (0); }

    //! Read the gripper angle in radian
    virtual int getGripperAngleRad(double& a_angle) { a_angle = 0; return (0); }

    //! Read the angular velocity of the gripper. Units are in radians per second [m/s].
    virtual int getGripperVelocity(double& a_gripperVelocity) { a_gripperVelocity = m_gripperVelocity; return (0); }

    //! Send a force [N] to the haptic device
    virtual int setForce(cVector3d& a_force) { return (0); }

    //! Read a sensed force [N] from the haptic device.
    virtual int getForce(cVector3d& a_force) { a_force = m_prevForce; return (0); }

    //! Send a torque [N*m] to the haptic device
    virtual int setTorque(cVector3d& a_torque) { return (0); }

    //! Read a sensed torque [N*m] from the haptic device
    virtual int getTorque(cVector3d& a_torque) { a_torque = m_prevTorque; return (0); }

    //! Send a torque [N*m] to the gripper
    virtual int setGripperTorque(double a_gripperTorque) { return (0); }

    //! Read a sensed torque [N*m] from the gripper
    virtual int getGripperTorque(double a_gripperTorque) { a_gripperTorque = m_prevGripperTorque; return (0); }

    //! Send a force [N], a torque [N*m] and a gripper torque [N*m] to the haptic device.
    virtual int setForceAndTorqueAndGripper(cVector3d& a_force, cVector3d& a_torque, double a_gripperTorque);

    //! read the status of the user switch [1 = ON / 0 = OFF]
    virtual int getUserSwitch(int a_switchIndex, int& a_status) { a_status = 0; return (0); }


    // METHODS:
    //! Get the specifications of the current device
    cHapticDeviceInfo getSpecifications() { return (m_specifications); }


  protected:

    //! Technical specifications of the current haptic device.
    cHapticDeviceInfo m_specifications;

    //! Previous sent force to the haptic device .
    cVector3d m_prevForce;

    //! Previous sent torque to the haptic device.
    cVector3d m_prevTorque;

    //! Previous sent gripper torque to the haptic device.
    double m_prevGripperTorque;

    //! Previous position of haptic device.
    cVector3d m_prevPosition;

    //! Previous orientation of haptic device.
    cMatrix3d m_prevRotation;

    //! Previous gripper position in radian.
    double m_prevGripperPosition;

    //! Last estimated linear velocity.
    cVector3d m_linearVelocity;

    //! Last estimated angular velocity.
    cVector3d m_angularVelocity;

    //! Last estimated gripper velocity.
    double m_gripperVelocity;

    //! Precision clock to measure linear velocity.
    cPrecisionClock clockLinearVelocity;

    //! Precision clock to measure angular velocity.
    cPrecisionClock clockAngularVelocity;

    //! Precision clock to measure velocity of the gripper.
    cPrecisionClock clockGripperVelocity;

    //! Estimate the linear velocity by passing the latest position.
    void estimateLinearVelocity(cVector3d& a_newPosition);

    //! Estimate the angular velocity by passing the latest orientation frame.
    void estimateAngularVelocity(cMatrix3d& a_newRotation);

    //! Estimate the velocity of the gripper by passing the latest gripper position.
    void estimateGripperVelocity(double a_newGripperPosition);

};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
