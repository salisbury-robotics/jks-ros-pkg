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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 699 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "tools/CGenericTool.h"
//---------------------------------------------------------------------------

//==========================================================================
/*!
    Constructor of cGenericTool.

    \fn     cGenericTool::cGenericTool(cWorld* a_parentWorld)
    \param  a_parentWorld  Parent world in which the tool belongs.
*/
//===========================================================================
cGenericTool::cGenericTool(cWorld* a_parentWorld)
{
    // set parent world
    m_parentWorld = a_parentWorld;

    // no device is currently connected to this tool
    m_device = NULL;

    // tool is not yet enabled
    m_enabled = false;

    // make sure that the simulation starts sending forces to the device
    // only when they are very small.
    m_waitForSmallForce = true;
	m_forceStarted		= false;
	m_forceON			= true;

    // initialize all members
    m_userSwitches              = 0;
    m_lastComputedGlobalForce   = cVector3d(0,0,0);
    m_lastComputedLocalForce    = cVector3d(0,0,0);
    m_lastComputedGlobalTorque  = cVector3d(0,0,0);
    m_lastComputedLocalTorque   = cVector3d(0,0,0);
    m_lastComputedGripperForce  = 0.0;
    m_workspaceRadius           = 1.0;
    m_workspaceScaleFactor      = 1.0;
    m_deviceLocalPos            = cVector3d(0,0,0);
    m_deviceGlobalPos           = cVector3d(0,0,0);
    m_deviceLocalRot            = cIdentity3d();
    m_deviceGlobalRot           = cIdentity3d();
    m_deviceLocalLinVel         = cVector3d(0,0,0);
    m_deviceGlobalLinVel        = cVector3d(0,0,0);
    m_deviceLocalRotVel         = cVector3d(0,0,0);
    m_deviceGlobalRotVel        = cVector3d(0,0,0);
    m_deviceGripperAngle        = 0.0;

    // clear contact points
    m_interactionPoints.clear();

    // clear contact events
    m_collisionEvents.clear();

    // clear interaction events
    m_interactionEvents.clear();

    // create a mesh for tool display purposes
    m_image = new cMesh(m_parentWorld);
}


//===========================================================================
/*!
    Set radius size of all contact points. The radius affects the physical 
    radius of the proxy and spheres used to render the goal and proxy 
    positions.

    \fn     void cGenericTool::setRadius(double a_radius)
    \param  a_radius  New radius for display and contact computation.
*/
//===========================================================================
void cGenericTool::setRadius(double a_radius)
{
    for (unsigned int i=0; i<m_interactionPoints.size(); i++)
    {
        m_interactionPoints[i]->setRadius(a_radius);
    }
}


//===========================================================================
/*!
   Set radius size of the display spheres (goal and proxy) and physical contact 
   sphere (proxy) used to compute the contact forces. Setting the a_radiusContact
   parameter to zero will generally accelerate the force rendering algorithm.

    \fn     void cGenericTool::setRadius(double a_radiusDisplay, 
                                              double a_radiusContact)
    \param  a_radiusDisplay  New radius for display of spheres (proxy and goal).
    \param  a_radiusContact  New radius for contact computation (proxy).
*/
//===========================================================================
void cGenericTool::setRadius(double a_radiusDisplay, double a_radiusContact)
{
    for (unsigned int i=0; i< m_interactionPoints.size(); i++)
    {
        m_interactionPoints[i]->setRadius(a_radiusDisplay, a_radiusContact);
    }
}


//===========================================================================
/*!
    Set radius size of the physical proxy. The change is affect to all
    contact points composing the tool.

    \fn     void cGenericTool::setRadiusContact(double a_radiusContact)
    \param  a_radiusContact  New radius for contact computation (proxy).
*/
//===========================================================================
void cGenericTool::setRadiusContact(double a_radiusContact)
{
    for (unsigned int i=0; i< m_interactionPoints.size(); i++)
    {
        m_interactionPoints[i]->setRadiusContact(a_radiusContact);
    }
}


//===========================================================================
/*!
    Set radius size of the sphere used to display the proxy and goal position.

    \fn     void cGenericTool::setRadiusDisplay(double a_radiusDisplay)
    \param  a_radiusDisplay  New radius for display of spheres (proxy and goal).
*/
//===========================================================================
void cGenericTool::setRadiusDisplay(double a_radiusDisplay)
{
    for (unsigned int i=0; i< m_interactionPoints.size(); i++)
    {
        m_interactionPoints[i]->setRadiusDisplay(a_radiusDisplay);
    }
}


//===========================================================================
/*!
    Set display options of the goal and proxy spheres. If both spheres are 
    enabled, a small line is drawn between both spheres.

    \fn     void cGenericTool::setShowContactPoints(bool a_showProxy, bool a_showGoal)
    \param  a_showProxy If \b true, then the proxy sphere is displayed.
    \param  a_showGoal If \b true, then the goal sphere is displayed.

*/
//===========================================================================
void cGenericTool::setShowContactPoints(bool a_showProxy, bool a_showGoal)
{
    for (unsigned int i=0; i< m_interactionPoints.size(); i++)
    {
        m_interactionPoints[i]->setShow(a_showProxy, a_showGoal);
    }
}


//===========================================================================
/*!
    Enable or disable the dynamic proxy algorithm to support dynamic objects.
    This option must be enabled if you have cMesh object which move inside
    the world and collide with your tool. However, if your world contains
    only static cMesh objects or potential field objects, then this option can 
    be disabled. Not enabling the dynamic proxy model for dynamic object will
    create "pop through" situation when the tool traverses the mesh without 
    detecting any collision.

    \fn     void cGenericTool::setShowContactPoints(bool a_showProxy, bool a_showGoal)
    \param  a_showProxy If \b true, then the proxy sphere is displayed.
    \param  a_showGoal If \b true, then the goal sphere is displayed.

*/
//===========================================================================
void cGenericTool::enableDynamicObjects(bool a_enabled)
{
    for (unsigned int i=0; i< m_interactionPoints.size(); i++)
    {
        m_interactionPoints[i]->m_algorithmFingerProxy->m_useDynamicProxy = a_enabled;
    }
}


//==========================================================================
/*!
    Read the status of the user switches from the haptic device
    controlled by this tool.

    \fn       bool cGenericTool::getUserSwitch(int a_switchIndex)
    \param    a_switchIndex Index number of the switch.
    \return   Return \b true if switch is active, otherwise return \b false.
*/
//===========================================================================
bool cGenericTool::getUserSwitch(int a_switchIndex)
{
    // check if tool is currently enabled
    if (!m_enabled) { return (false); }

    // read selected user switch
	bool userSwitch = false;
    
	// check switch
	if (m_device != NULL)
	{
		m_device->getUserSwitch(a_switchIndex, userSwitch);
	}

	// return result
    return (userSwitch);
}


//==========================================================================
/*!
    Start the haptic tool. A connection is opened to the haptic device and
    the position of the tool initialized by reading the initial position
    of the haptic device

    \fn       int cGenericTool::start()
    \return   0 indicates success, non-zero indicates an error
*/
//===========================================================================
int cGenericTool::start()
{
    // check if device is available
    if (m_device == NULL)
    {
        return -1;
    }

    // open connection to device
    if (m_device->open() == 0)
    {
        m_enabled = true;
    }
    else
    {
        m_enabled = false;
        return (-1);
    }

    // update position
    updatePose();

    // intialize tool by resetting the force models
    initialize();

	// resturn result
	return (0);
}


//==========================================================================
/*!
    Stop the haptic tool. The connection to the haptic device is closed

    \fn       void cToolCursor::stop()
    \return   0 indicates success, non-zero indicates an error
*/
//===========================================================================
int cGenericTool::stop()
{
    // check if tool is currently enabled
    if (!m_enabled) { return (-1); }

    // stop the device
    return (m_device->close());
}


//===========================================================================
/*!
    Reset all force models according to the current position of the 
    haptic device.

    \fn     void cGenericTool::initialize()
*/
//===========================================================================
void cGenericTool::initialize()
{
    // update position from haptic device
    updatePose();    

    // initialize all contact points
    for (unsigned int i=0; i< m_interactionPoints.size(); i++)
    {
        m_interactionPoints[i]->initialize(m_deviceGlobalPos);
    }
}


//==========================================================================
/*!
    Update position of pointer and orientation of wrist.

    \fn       void cGenericTool::updatePose()
*/
//===========================================================================
void cGenericTool::updatePose()
{
    // check if device is available
    if (m_device == NULL) { return; }

    //////////////////////////////////////////////////////////////////////
    // retrieve data from haptic device
    //////////////////////////////////////////////////////////////////////

    // temp variables
    cVector3d devicePos, deviceLinVel, deviceRotVel;
    cMatrix3d deviceRot;
    double deviceGripperAngle;

    // init temp variable
    devicePos.zero();
    deviceLinVel.zero();
    deviceRotVel.zero();
    deviceRot.identity();
    deviceGripperAngle = 0.0;

    // update position, orientation, linear and rotational velocities from device
    m_device->getPosition(devicePos);
    m_device->getRotation(deviceRot);
    m_device->getGripperAngleRAD(deviceGripperAngle);
    m_device->getLinearVelocity(deviceLinVel);


    //////////////////////////////////////////////////////////////////////
    // update information inside tool
    //////////////////////////////////////////////////////////////////////

    // compute local position - adjust for tool workspace scale factor
    m_deviceLocalPos = m_workspaceScaleFactor * devicePos;

    // compute global position in world coordinates
    cVector3d pos;
    m_globalRot.mulr(m_deviceLocalPos, pos);
    pos.addr(m_globalPos, m_deviceGlobalPos);

    // compute local rotation
    m_deviceLocalRot = deviceRot; 

    // compute global rotation
    m_deviceLocalRot.mulr(m_globalRot, m_deviceGlobalRot);

    // compute local linear velocity - adjust for tool workspace scale factor
    m_deviceLocalLinVel = m_workspaceScaleFactor * deviceLinVel;

    // compute global linear velocity
    m_globalRot.mulr(m_deviceLocalLinVel, m_deviceGlobalLinVel);

    // compute local rotational velocity
    m_deviceLocalRotVel = deviceRotVel;

    // compute global rotational velocity
    m_globalRot.mulr(m_deviceLocalRotVel, m_deviceGlobalRotVel);

    // compute gripper angle
    m_deviceGripperAngle = deviceGripperAngle;

    // read user switch status
    m_userSwitch0 = getUserSwitch(0);

    // update the position and orientation of the tool image
    updateToolImagePosition();
}


//===========================================================================
/*!
    Update the position and orientation of the tool image.

    \fn       void cGenericTool::computeInteractionForces()
*/
//===========================================================================
void cGenericTool::updateToolImagePosition()
{
    // set the position and orientation of the tool image to be equal to the 
    // one of the haptic device. Under more complexe tools, these values could 
    // typically be function of the combined positions of various contact 
    // points (or proxy positions).
    m_image->setPos(m_deviceLocalPos);
    m_image->setRot(m_deviceLocalRot);
}


//===========================================================================
/*!
    Compute the interaction forces between the tool and the virtual
    object inside the virtual world.

    \fn       void cGenericTool::computeInteractionForces()
*/
//===========================================================================
void cGenericTool::computeInteractionForces()
{
    // for each contact point contact point compute the interaction force
    // and combine their overall contribution to compute the output force
    // and torque to be sent to the haptic device

    // initialize variables
    cVector3d force, torque;
    force.zero();
    torque.zero();

    int numContactPoint = (int)(m_interactionPoints.size());
    for (int i=0; i<numContactPoint; i++)
    {
        // get next contact point
        cInteractionPoint* nextContactPoint = m_interactionPoints[i];

        // compute force at contact point as well as new proxy position
        cVector3d t_force = nextContactPoint->computeInteractionForces(m_deviceGlobalPos, m_deviceGlobalLinVel);
        cVector3d t_pos = nextContactPoint->getGlobalPosProxy();

        // combine force contributions together
        force.add(t_force);
        torque.add(cCross(t_pos, t_force));
    }

    // update global forces
    m_lastComputedGlobalForce  = force;
    m_lastComputedGlobalTorque = torque;
    m_lastComputedGripperForce = 0.0;

    // update coolision and interaction event lists
    updateCollisionAndInteractionEventLists();
}


//==========================================================================
/*!
    This method dhould be called inside the computeInteractionForces() method
    to update the list of interaction and collision events that occured 
    between the tool contact points and the environment.

    \fn       void cGenericTool::updateCollisionAndInteractionEventLists()
*/
//===========================================================================
void cGenericTool::updateCollisionAndInteractionEventLists()
{
    // clear lists
    m_collisionEvents.clear();
    m_interactionEvents.clear();

    // update lists
    for (unsigned int i=0; i< m_interactionPoints.size(); i++)
    {
        //////////////////////////////////////////////////
        // collision events
        //////////////////////////////////////////////////
        cInteractionPoint* contactPoint = m_interactionPoints[i];

        for (int i=0; i<3; i++)
        {
            if (contactPoint->m_algorithmFingerProxy->m_contacts[i]->m_object != NULL)
            {
                m_collisionEvents.push_back(contactPoint->m_algorithmFingerProxy->m_contacts[i]);
            }
        }


        //////////////////////////////////////////////////
        // interaction events
        //////////////////////////////////////////////////
        for (unsigned int j=0; j<contactPoint->m_algorithmPotentialField->m_interactionRecorder.m_interactions.size(); j++)
        {
            m_interactionEvents.push_back(&(contactPoint->m_algorithmPotentialField->m_interactionRecorder.m_interactions[j]));
        }
    }
}


//==========================================================================
/*!
    Apply the latest computed force to the haptic device.

    \fn       void cGenericTool::applyForces()
*/
//===========================================================================
void cGenericTool::applyForces()
{
    // check if device is available
    if (m_device == NULL)
    {
        return;
    }

    // compute local forces from global forces compute in world coordinates
    cMatrix3d rot;
    m_globalRot.transr(rot);
    rot.mulr(m_lastComputedGlobalForce, m_lastComputedLocalForce);
    rot.mulr(m_lastComputedGlobalTorque, m_lastComputedLocalTorque);

    // verification before engaging forces the first time
    if ((m_waitForSmallForce == false)  ||
        ((!m_forceStarted) && (m_lastComputedGlobalForce.lengthsq() <0.000001)))
    {
        m_forceStarted = true;
    }

    // send force commands to haptic device
    if ((m_forceON) && (m_forceStarted))
    {
        m_device->setForceAndTorqueAndGripperForce(m_lastComputedLocalForce, 
                                                   m_lastComputedLocalTorque, 
                                                   m_lastComputedGripperForce);
    }
    else
    {
        cVector3d nullv3d (0.0, 0.0, 0.0);
        m_device->setForceAndTorqueAndGripperForce(nullv3d,
                                                   nullv3d,
                                                   0.0);
    }
}


//==========================================================================
/*!
    Turns forces \b ON.

    \fn     void cGenericTool::setForcesON()
    \return   0 indicates success, non-zero indicates an error

*/
//===========================================================================
int cGenericTool::setForcesON()
{
    // flag forces as ON
    if (!m_forceON)
    {
          m_forceStarted = false;
          m_forceON = true;
    }

    // return success
    return (0);
}


//==========================================================================
/*!
    Turns forces \b OFF.

    \fn       void cGenericTool::setForcesOFF()
    \return   0 indicates success, non-zero indicates an error
*/
//===========================================================================
int cGenericTool::setForcesOFF()
{
    // flag force as OFF
    m_forceON = false;
    
    // sett all force variables to zero.
    m_lastComputedLocalForce.zero();
    m_lastComputedGlobalForce.zero();
    m_lastComputedLocalTorque.zero();
    m_lastComputedGlobalTorque.zero();
    m_lastComputedGripperForce = 0.0;

    // send zero forces to haptic device
    applyForces();

    // return success
    return (0);
}


//==========================================================================
/*!
    Check if the tool is currently interacting with the given object.

    \fn       bool cGenericTool::isInContact(cGenericObject* a_object)
    \return   Return \e true if the tool is interacting with the object
*/
//===========================================================================
bool cGenericTool::isInContact(cGenericObject* a_object)
{
    for (unsigned int i=0; i<m_interactionPoints.size(); i++)
    {
        if (m_interactionPoints[i]->isInContact(a_object))
        {
            return (true);
        }
    }
    return (false);
}


//==========================================================================
/*!
    Sets the virtual volume in which the virtual tool will be moving.

    \fn       void cGenericTool::setWorkspaceRadius(const double& a_workspaceRadius)
    \param    a_workspaceRadius   Radius of the workspace.
*/
//===========================================================================
void cGenericTool::setWorkspaceRadius(const double& a_workspaceRadius)
{
    // update new workspace size
    m_workspaceRadius = a_workspaceRadius;

    // compute the new scale factor between the workspace of the tool
    // and one of the haptic device
    if (m_device != NULL)
    {
        m_workspaceScaleFactor = m_workspaceRadius / m_device->getSpecifications().m_workspaceRadius;
    }
    else
    {
        m_workspaceScaleFactor = 1.0;
    }

    // since the workspace has changed, the position of the tool will change
    // too therfeore it is necessary to re-initialize the contact models
    // in order to avoid a possible force spike coming from the proxy models.
    initialize();
}


//==========================================================================
/*!
    Define a scale factor between the physical workspace of the haptic device
    and the workspace span by the virtual tool.

    \fn       void cGenericTool::setWorkspaceScaleFactor(const double& a_workspaceScaleFactor)
    \param    a_workspaceScaleFactor   Workspace scale factor.
*/
//===========================================================================
void cGenericTool::setWorkspaceScaleFactor(const double& a_workspaceScaleFactor)
{
    // make sure that input value is bigger than zero
    double value = cAbs(a_workspaceScaleFactor);
    if (value == 0) { return; }

    // update scale factor
    m_workspaceScaleFactor = value;

    // compute the new scale factor between the workspace of the tool
    // and one of the haptic device
    m_workspaceRadius =  m_workspaceScaleFactor * m_device->getSpecifications().m_workspaceRadius;

    // since the workspace has changed, the position of the tool will change
    // too therfeore it is necessary to re-initialize the contact models
    // in order to avoid a possible force spike coming from the proxy models.
    initialize();
}
