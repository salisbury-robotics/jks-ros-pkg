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
#include "devices/CGenericHapticDevice.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cGenericHapticDevice.
    Initialize basic parameters of generic haptic device.

    \fn     cGenericHapticDevice::cGenericHapticDevice()
*/
//===========================================================================
cGenericHapticDevice::cGenericHapticDevice() : cGenericDevice()
{
    m_specifications.m_manufacturerName              = "not defined";
    m_specifications.m_modelName                     = "not defined";
    m_specifications.m_maxForce                      = 0.1; // [N]
    m_specifications.m_maxForceStiffness             = 0.1; // [N/m]
    m_specifications.m_maxTorque                     = 0.1; // [N*m]
    m_specifications.m_maxTorqueStiffness            = 0.1; // [N*m/Rad]
    m_specifications.m_maxGripperTorque              = 0.1; // [N]
    m_specifications.m_maxGripperTorqueStiffness     = 0.1; // [N*m/m]
    m_specifications.m_workspaceRadius               = 0.1; // [m]
    m_specifications.m_sensedPosition                = false;
    m_specifications.m_sensedRotation                = false;
    m_specifications.m_sensedGripper                 = false;
    m_specifications.m_actuatedPosition              = false;
    m_specifications.m_actuatedRotation              = false;
    m_specifications.m_actuatedGripper               = false;
    m_specifications.m_leftHand                      = false;
    m_specifications.m_rightHand                     = false;

    m_prevForce.zero();
    m_prevTorque.zero();
    m_prevGripperTorque = 0.0;

    m_prevPosition.zero();
    m_prevRotation.identity();
    m_prevGripperPosition = 0.0;

    m_linearVelocity.zero();
    m_angularVelocity.zero();
    m_gripperVelocity = 0.0;

    clockLinearVelocity.reset();
    clockAngularVelocity.reset();
    clockGripperVelocity.reset();
}


//===========================================================================
/*!
    Set command for the haptic device

    \fn         int cGenericHapticDevice::command(int a_command, void* a_data)
    \param      a_command  Selected command.
    \param      a_data  Pointer to the corresponding data structure.
    \return     Return status of command.
*/
//===========================================================================
int cGenericHapticDevice::command(int a_command, void* a_data)
{
    // temp variables
    int result;

    // check if the device is open
    if (m_systemReady)
    {
        switch (a_command)
        {
            // read position of end-effector
            case CHAI_CMD_GET_POS_3D:
            {
                cVector3d temp;
                result = getPosition(temp);
                cVector3d* position = (cVector3d *) a_data;
                *position = temp;
            }
            break;

            // read normalized position of end-effector
            case CHAI_CMD_GET_POS_NORM_3D:
            {
                cVector3d temp;
                result = getPosition(temp);
                temp.div(m_specifications.m_workspaceRadius);
                cVector3d* position = (cVector3d *) a_data;
                *position = temp;
            }
            break;

            // read orientation of end-effector
            case CHAI_CMD_GET_ROT_MATRIX:
            {
                cMatrix3d temp;
                result = getRotation(temp);
                cMatrix3d* rotation = (cMatrix3d *) a_data;
                *rotation = temp;
            }
            break;

            // set force to end-effector
            case CHAI_CMD_SET_FORCE_3D:
            {
                cVector3d* force = (cVector3d *) a_data;
                result = setForce(*force);
            }
            break;

            // set torque to end-effector
            case CHAI_CMD_SET_TORQUE_3D:
            {
                cVector3d* torque = (cVector3d *) a_data;
                result = setTorque(*torque);
            }
            break;

            // read user switch from end-effector
            case CHAI_CMD_GET_SWITCH_0:
            {
                int* status = (int *) a_data;
                int temp;
                result = getUserSwitch(0, temp);
                *status = temp;
            }
            break;

            // read user switch from end-effector
            case CHAI_CMD_GET_SWITCH_MASK:
            {
                // Force the result to be 0 or 1, since bit 0 should carry button 0's value
                int* status = (int *) a_data;
                int temp;
                result = getUserSwitch(0, temp);
                *status = temp ? 1 : 0;
            }
            break;

            // function is not implemented
            default:
                result = CHAI_MSG_NOT_IMPLEMENTED;
        }
    }
    else
    {
        result = CHAI_MSG_SYSTEM_NOT_READY;
    }
    return (result);
}


//===========================================================================
/*!
    Send a force [N] and a torque [N*m] and gripper force [N] to the
    haptic device.

    \fn     int cGenericHapticDevice::setForceAndTorqueAndGripper(cVector3d& a_force,
            cVector3d& a_torque, double a_gripperTorque)
    \param  a_force  Force command.
    \param  a_torque  Torque command.
    \param  a_gripperTorque  Gripper torque command.
    \return Return 0 if no error occured.
*/
//===========================================================================
int cGenericHapticDevice::setForceAndTorqueAndGripper(cVector3d& a_force, cVector3d& a_torque,
                  double a_gripperTorque)
{
    int error0, error1, error2;

    // send force command
    error0 = setForce(a_force);

    // send torque command
    error1 = setTorque(a_force);

    // send gripper command
    error2 = setGripperTorque(a_gripperTorque);

    // return status
    if ((error0 != 0) || (error1 != 0) || (error2 != 0))
    {
        // an error has occured
        return (-1);
    }
    else
    {
        // success
        return (0);
    }
}


//===========================================================================
/*!
    Estimate the linear velocity by passing the latest position.

    \fn     void cGenericHapticDevice::estimateLinearVelocity(cVector3d& a_newPosition)
    \param  a_newPosition  New position of the device.
    \return Return 0 if no error occured.
*/
//===========================================================================
void cGenericHapticDevice::estimateLinearVelocity(cVector3d& a_newPosition)
{
    // time limits
    const double TIME_MIN = 0.02;  // [s]
    const double TIME_MAX = 0.10;  // [s]


    // read clock in seconds
    double time = clockLinearVelocity.stop();

    // check time
    if (time < TIME_MIN) { return; }

    // check if previous position remains the same
    if ((m_prevPosition.equals(a_newPosition)) && (time < TIME_MAX))
    {
        return;
    }

    // compute velocity
    if (time == 0)
    {
        m_linearVelocity.zero();
    }
    else
    {
        cVector3d result;
        a_newPosition.subr(m_prevPosition, result);
        result.divr(time, m_linearVelocity);
    }

    // store new value
    m_prevPosition = a_newPosition;

    // restart clock
    clockLinearVelocity.reset();
    clockLinearVelocity.start();
}


//===========================================================================
/*!
    Estimate the angular velocity by passing the latest orientation frame.

    \fn     void cGenericHapticDevice::estimateAngularVelocity(cMatrix3d& a_newRotation)
    \param  a_newRotation  New orientation frame of the device.
*/
//===========================================================================
void cGenericHapticDevice::estimateAngularVelocity(cMatrix3d& a_newRotation)
{
    // read clock in seconds
    double time = clockAngularVelocity.stop();

    // check if previous position remains the same
    const double TIME_MAX = 0.01;  // [s]
    if ((m_prevRotation.equals(a_newRotation)) && (time < TIME_MAX))
    {
        return;
    }

    // compute velocity
    if (time == 0)
    {
        m_angularVelocity.zero();
    }
    else
    {
        // TODO: COMPUTE ANGULAR VELOCITY BY LOOKING AT TWO FRAMES
        // LET'S PUT ZERO FOR NOW.
        m_angularVelocity.zero();
    }

    // store new value
    m_prevRotation = a_newRotation;

    // restart clock
    clockAngularVelocity.reset();
    clockAngularVelocity.start();
}


//===========================================================================
/*!
    Estimate the velocity of the gripper by passing the latest gripper position.

    \fn     void cGenericHapticDevice::estimateGripperVelocity(double a_newGripperPosition)
    \param  a_newGripperPosition  New position of the gripper.
*/
//===========================================================================
void cGenericHapticDevice::estimateGripperVelocity(double a_newGripperPosition)
{
    // read clock in seconds
    double time = 0.000001 * (double)clockGripperVelocity.stop();

    // compute velocity
    if (time == 0)
    {
        m_gripperVelocity = 0.0;
    }
    else
    {
        m_gripperVelocity = (a_newGripperPosition - m_prevGripperPosition) / time;
    }

    // store new value
    m_prevGripperPosition = a_newGripperPosition;

    // restart clock
    clockGripperVelocity.reset();
    clockGripperVelocity.start();
}

