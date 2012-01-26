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
    \author    Federico Barbagli
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 365 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "tools/CToolGripper.h"
#include "graphics/CTriangle.h"
//---------------------------------------------------------------------------

//==========================================================================
/*!
    Constructor of cToolGripper.

    \fn       cToolGripper::cToolGripper(cWorld* a_parentWorld):cGenericTool(a_parentWorld)
    \param    a_world  World in which the tool will operate.
*/
//===========================================================================
cToolGripper::cToolGripper(cWorld* a_parentWorld):cGenericTool(a_parentWorld)
{
    // create a contact point contact for the thumb
    m_interactionPointThumb = new cInteractionPoint(this);
    m_interactionPoints.push_back(m_interactionPointThumb);

    // create a contact point contact for the finger
    m_interactionPointFinger = new cInteractionPoint(this);
    m_interactionPoints.push_back(m_interactionPointFinger);

    // show proxy spheres only
    setShowContactPoints(true, false);
}


//==========================================================================
/*!
    Destructor of cToolGripper.

    \fn       cToolGripper::~cToolGripper()
*/
//===========================================================================
cToolGripper::~cToolGripper()
{
    delete  m_interactionPointThumb;
    delete  m_interactionPointFinger;
}


//===========================================================================
/*!
    Compute the interaction forces between the tool and the virtual
    object inside the virtual world.

    \fn       void cToolGripper::computeInteractionForces()
*/
//===========================================================================
void cToolGripper::computeInteractionForces()
{
    // compute new position of thumb and finger 
    cVector3d lineFingerThumb = getGlobalRot().getCol1();    
    cVector3d posFinger = m_deviceGlobalPos + cMul(m_deviceGlobalRot, (  0.6 * m_deviceGripperAngle * lineFingerThumb));
    cVector3d posThumb = m_deviceGlobalPos +  cMul(m_deviceGlobalRot, (- 0.6 * m_deviceGripperAngle * lineFingerThumb));
   // cVector3d posFinger = (m_deviceGlobalPos + 0.3 * 0.4 * lineFingerThumb);
   // cVector3d posThumb = (m_deviceGlobalPos - 0.3 * 0.4 * lineFingerThumb);

    // compute force interaction forces
    cVector3d force0 = m_interactionPointThumb->computeInteractionForces(posThumb, m_deviceGlobalLinVel);
	cVector3d force1 = m_interactionPointFinger->computeInteractionForces(posFinger, m_deviceGlobalLinVel);
    
    double gripperForce = 0.0;
	cVector3d dir = posFinger - posThumb;
    if (dir.length() > 0.00001) 
	{
		dir.normalize ();
		cVector3d force = cProject (force1, dir);
		gripperForce = force.length();
		if (force.length() > 0.001) 
		{
			double angle = cAngle(dir, force);
			if ((angle > C_PI/2.0) || (angle < -C_PI/2.0)) gripperForce = -gripperForce;
		}
    }

	cVector3d torque = 0.2 * cAdd(cCross(cSub(posThumb, m_deviceGlobalPos), force0), cCross(cSub(posFinger, m_deviceGlobalPos), force1));

    m_lastComputedGlobalForce = force0 + force1;
    m_lastComputedGlobalTorque = torque;

	double gripperVelocity = 0.0;
	m_device->getGripperAngularVelocity(gripperVelocity);
	m_lastComputedGripperForce = gripperForce - 0.2 * gripperVelocity;

    // DLR PATCH!
    //m_lastComputedGripperForce = -m_lastComputedGripperForce;

    // update coolision and interaction event lists
    updateCollisionAndInteractionEventLists();
}


//==========================================================================
/*!
    Render the current tool in OpenGL.

    \fn       void cToolGripper::render(cRenderOptions& a_options)
    \param    a_options  Rendering options.
*/
//===========================================================================
void cToolGripper::render(cRenderOptions& a_options)
{
    ///////////////////////////////////////////////////////////////////////
    // render contact points
    ///////////////////////////////////////////////////////////////////////
    int numContactPoint = (int)(m_interactionPoints.size());
    for (int i=0; i<numContactPoint; i++)
    {
        // get next contact point
        cInteractionPoint* nextContactPoint = m_interactionPoints[i];

        // render tool
        nextContactPoint->render(a_options);
    }

    ///////////////////////////////////////////////////////////////////////
    // render mesh image
    ///////////////////////////////////////////////////////////////////////
    if (m_image != NULL)
    {
        m_image->renderSceneGraph(a_options);    
    }
}

