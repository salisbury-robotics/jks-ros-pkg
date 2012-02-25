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
#include "tools/CToolBody.h"
#include "graphics/CTriangle.h"
//---------------------------------------------------------------------------

//==========================================================================
/*!
    Constructor of cToolBody.

    \fn       cToolBody::cToolBody(cWorld* a_parentWorld):cGenericTool(a_parentWorld)
    \param    a_world  World in which the tool will operate.
*/
//===========================================================================
cToolBody::cToolBody(cWorld* a_parentWorld):cGenericTool(a_parentWorld)
{
    // create a single point contact
    m_interactionPoint = new cInteractionPoint(this);

    // add point to list
    m_interactionPoints.push_back(m_interactionPoint);

    // show proxy spheres only
    setShowContactPoints(true, false);
}


//==========================================================================
/*!
    Destructor of cToolBody.

    \fn       cToolBody::~cToolBody()
*/
//===========================================================================
cToolBody::~cToolBody()
{
    delete m_interactionPoint;
}


//===========================================================================
/*!
    Update the position and orientation of the tool image.

    \fn       void cToolBody::computeInteractionForces()
*/
//===========================================================================
void cToolBody::updateToolImagePosition()
{
    // set the position and orientation of the tool image to be equal to the 
    // one of the contact point proxy.
    cVector3d pos = m_interactionPoint->getLocalPosProxy();
    m_image->setPos(pos);
    m_image->setRot(m_deviceLocalRot);
}


//===========================================================================
/*!
    Compute the interaction forces between the cursor and the virtual
    environment.

    \fn       void cToolBody::computeInteractionForces()
*/
//===========================================================================
void cToolBody::computeInteractionForces()
{
    // compute force interaction forces at contact point
    m_lastComputedGlobalForce = m_interactionPoint->computeInteractionForces(m_deviceGlobalPos, m_deviceGlobalLinVel);
    m_lastComputedGlobalTorque.zero();
    m_lastComputedGripperForce = 0.0;

    // update coolision and interaction event lists
    updateCollisionAndInteractionEventLists();
}


//==========================================================================
/*!
    Render the current tool in OpenGL.

    \fn       void cToolBody::render(cRenderOptions& a_options)
    \param    a_options  Rendering options.
*/
//===========================================================================
void cToolBody::render(cRenderOptions& a_options)
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




