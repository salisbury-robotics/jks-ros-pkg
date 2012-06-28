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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 794 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "CODEWorld.h"
//---------------------------------------------------------------------------
//! Maximum number of contact points per body.
#define MAX_CONTACTS_PER_BODY 16  
//---------------------------------------------------------------------------


//==========================================================================
/*!
      Constructor of cODEWorld.

      \fn       cODEWorld::cODEWorld(cWorld* a_parentWorld)
      \param    a_parentWorld  Pointer to parent CHAI3D world.
      \return   Return pointer to new cODEWorld instance.
*/
//===========================================================================
cODEWorld::cODEWorld(cWorld* a_parentWorld)
{
    // init ODE
	dInitODE();

	// set parent world
	m_parentWorld = a_parentWorld;

    // reset simulation time.
    m_simulationTime = 0.0;

    // create ODE world
    m_ode_world = dWorldCreate();

	// create ODE space
	m_ode_space = dHashSpaceCreate(0);

	// setup callback to handle collision
	dSpaceCollide(m_ode_space, this, &(cODEWorld::nearCallback));

	// create ODE contact group
	m_ode_contactgroup = dJointGroupCreate(0);

	// set some default damping parameters
	dWorldSetLinearDamping(m_ode_world, 0.00001);
	dWorldSetAngularDamping(m_ode_world, 0.0015);
	dWorldSetMaxAngularSpeed(m_ode_world, 200);
}


//===========================================================================
/*!
      Destructor of cODEWorld.

      \fn       cODEWorld::~cODEWorld()
*/
//===========================================================================
cODEWorld::~cODEWorld()
{
    // clear all bodies
    m_bodies.clear();

    // cleanup ODE
	dJointGroupDestroy(m_ode_contactgroup);
	dSpaceDestroy(m_ode_space);
	dWorldDestroy(m_ode_world);
}


//===========================================================================
/*!
      Define a gravity field.

      \fn       void cODEWorld::setGravity(cVector3d a_gravity)
      \param    a_gravity  Gravity field.
*/
//===========================================================================
void cODEWorld::setGravity(cVector3d a_gravity)
{
    // update ode
    dWorldSetGravity (m_ode_world, a_gravity.x(), a_gravity.y(), a_gravity.z());
}


//===========================================================================
/*!
      Read the current gravity field.

      \fn       cVector3d cODEWorld::getGravity()
      \return   Return current gravity field.
*/
//===========================================================================
cVector3d cODEWorld::getGravity()
{
    // temp
    cVector3d result;
    dVector3 gravity;

    // return result
    dWorldGetGravity (m_ode_world, gravity);
    result.set(gravity[0], gravity[1], gravity[2]);
    return (result);
}


//===========================================================================
/*!
     Render world in OpenGL.

     \fn       void cODEWorld::render(cRenderOptions& a_options)
     \param    a_options  Rendering options.
*/
//===========================================================================
void cODEWorld::render(cRenderOptions& a_options)
{
    list<cODEGenericBody*>::iterator i;

    // render all dynamic ODE bodies
    for(i = m_bodies.begin(); i != m_bodies.end(); ++i)
    {
        cODEGenericBody* nextItem = *i;
        nextItem->renderSceneGraph(a_options);
    }
}


//===========================================================================
/*!
    Compute globalPos and globalRot given the localPos and localRot
    of this object and its parents.  Optionally propagates to children.

    If \e a_frameOnly is set to \b false, additional global positions such as
    vertex positions are computed (which can be time-consuming).

     \fn       void cODEWorld::updateGlobalPositions(const bool a_frameOnly)
     \param    a_frameOnly  If \b true then only the global frame is computed.
*/
//===========================================================================
void cODEWorld::updateGlobalPositions(const bool a_frameOnly)
{
    list<cODEGenericBody*>::iterator i;

    for(i = m_bodies.begin(); i != m_bodies.end(); ++i)
    {
        cODEGenericBody* nextItem = *i;
        nextItem->computeGlobalPositions(a_frameOnly,
                                         m_globalPos,
                                         m_globalRot);
    }
};


//===========================================================================
/*!
      Compute simulation for a_time time interval.

      \fn       void cODEWorld::updateDynamics(double a_interval)
      \param    a_interval  Time increment.
*/
//===========================================================================
void cODEWorld::updateDynamics(double a_interval)
{
	// update collision callback information
	dSpaceCollide (m_ode_space, 0, &(cODEWorld::nearCallback));

    // integrate simulation during an certain interval
	// dWorldStep (m_ode_world, a_interval);
    // dWorldStepFast1 (m_ode_world, a_interval, 5);
    dWorldQuickStep (m_ode_world, a_interval);

    // cleanup contacts from previous iteration
	dJointGroupEmpty(m_ode_contactgroup);

    // add time to overall simulation
    m_simulationTime = m_simulationTime + a_interval;

    // update CHAI3D positions for of all object
    updateBodyPositions();
}


//===========================================================================
/*!
      Update position and orientation from ode models to chai models.

      \fn       void cODEWorld::updateBodyPositions(void)
*/
//===========================================================================
void cODEWorld::updateBodyPositions(void)
{
    list<cODEGenericBody*>::iterator i;

    for(i = m_bodies.begin(); i != m_bodies.end(); ++i)
    {
        cODEGenericBody* nextItem = *i;
        nextItem->updateBodyPosition();
    }
}


//===========================================================================
/*!
      Callback for handling collision detection.

      \fn       void cODEWorld::nearCallback (void *a_data, 
                                              dGeomID a_object1, 
                                              dGeomID a_object2)
      \param    a_data   Not used here.
      \param    a_object1  Reference to ODE object 1.
      \param    a_object2  Reference to ODE object 2.
*/
//===========================================================================
void cODEWorld::nearCallback (void *a_data, dGeomID a_object1, dGeomID a_object2)
{
    // retrieve body ID for each object. This value is defined unless the object
    // is static.
    dBodyID b1 = dGeomGetBody(a_object1);
    dBodyID b2 = dGeomGetBody(a_object2);

    // exit without doing anything if the two bodies are connected by a joint
    if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

    dContact contact[MAX_CONTACTS_PER_BODY];
    int n = dCollide (a_object1, a_object2, MAX_CONTACTS_PER_BODY,&(contact[0].geom),sizeof(dContact));
    if (n > 0) 
    {
        for (int i=0; i<n; i++) 
        {
            // define default collision properties (this section could be extended to support some ODE material class!)
            contact[i].surface.slip1 = 0.7;
            contact[i].surface.slip2 = 0.7;
            contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
            contact[i].surface.mu = 50;
            contact[i].surface.soft_erp = 0.90;
            contact[i].surface.soft_cfm = 0.10;

            // get handles on each body
            cODEGenericBody* ode_body = NULL;
            if (b1 != NULL)
            {
	            ode_body = (cODEGenericBody*)dBodyGetData (b1);
            }
            else if (b2 != NULL)
            {
                // if first object is static, use the second one. (both objects can not be static) 
                ode_body = (cODEGenericBody*)dBodyGetData (b2);
            }
                
            // create a joint following collision
            if (ode_body != NULL)
            {
                dJointID c = dJointCreateContact (ode_body->m_ODEWorld->m_ode_world,
									                ode_body->m_ODEWorld->m_ode_contactgroup,
									                &contact[i]);
                dJointAttach (c,
		                    dGeomGetBody(contact[i].geom.g1),
		                    dGeomGetBody(contact[i].geom.g2));
            }

        }
    }
}


//==========================================================================
/*!
    Compute collision detection between a segment and objects in the ODE world.

    \fn       bool cODEWorld::computeOtherCollisionDetection(cVector3d& a_segmentPointA,
                                                         cVector3d& a_segmentPointB,
                                                         cCollisionRecorder& a_recorder,
                                                         cCollisionSettings& a_settings)
    \param    a_segmentPointA  Start point of segment.  
    \param    a_segmentPointB  End point of segment.
    \param    a_recorder  Stores all collision events.
    \param    a_settings  Contains collision settings information.  
    \return   Return \b true if a collision has occurred.
*/
//===========================================================================
bool cODEWorld::computeOtherCollisionDetection(cVector3d& a_segmentPointA,
                                                        cVector3d& a_segmentPointB,
                                                        cCollisionRecorder& a_recorder,
                                                        cCollisionSettings& a_settings)
{
    // check each ODE body
    list<cODEGenericBody*>::iterator i;
    bool result = false;
    for(i = m_bodies.begin(); i != m_bodies.end(); ++i)
    {
        cODEGenericBody *nextItem = *i;
        bool collide = nextItem->computeCollisionDetection(a_segmentPointA, 
                                                           a_segmentPointB, 
                                                           a_recorder,
                                                           a_settings);
        if (collide) { result = true; }
    }

    // return result
    return (result);
}
