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
#ifndef CODEWorldH
#define CODEWorldH
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CODEGenericBody.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CODEWorld.h

    \brief 
    <b> ODE Module </b> \n 
    ODE World.
*/
//===========================================================================


//===========================================================================
/*!
    \class      cODEWorld
    \ingroup    ODE

    \brief      
    cODEWorld implements a virtual world to handle ODE based objects 
    (cODEGenericBody).
*/
//===========================================================================
class cODEWorld : public cGenericObject
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cODEWorld.
    cODEWorld(cWorld* a_parentWorld);

    //! Destructor of cODEWorld.
    virtual ~cODEWorld();

    // PROPERTIES:
    //! List of ODE dynamic bodies.
    list<cODEGenericBody*> m_bodies;


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Set gravity field.
    void setGravity(cVector3d a_gravity);

    //! Read gravity field.
    cVector3d getGravity();

    //! Set linear damping.
    void setLinearDamping(double a_value) { dWorldSetLinearDamping(m_ode_world, a_value); }

    //! Set angular damping.
    void setAngularDamping(double a_value) { dWorldSetAngularDamping(m_ode_world, a_value); } 

    //! Set max angular speed.
    void setMaxAngularSpeed(double a_value) { dWorldSetMaxAngularSpeed(m_ode_world, a_value); }

    //! compute dynamic simulation.
    void updateDynamics(double a_interval);

    //! update position and orientation from ode models to chai models.
    void updateBodyPositions(void);

    // update global position frames.
    void updateGlobalPositions(const bool a_frameOnly);

    //! Return the nearest triangle intersected by the given segment, if any.
    virtual bool computeOtherCollisionDetection(cVector3d& a_segmentPointA,
                                                cVector3d& a_segmentPointB,
                                                cCollisionRecorder& a_recorder,
                                                cCollisionSettings& a_settings);


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! ODE dynamics world.
    dWorldID m_ode_world;

    //! ODE collision space.
    dSpaceID m_ode_space;

	//! ODE contact group.
	dJointGroupID m_ode_contactgroup;


  private:

	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! current time of simulation.
    double m_simulationTime;

    //! Parent chai3d world.
	cWorld* m_parentWorld;


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

	//! ODE collision callback.
	static void nearCallback (void *data, dGeomID o1, dGeomID o2);

    //! Render deformable mesh (OpenGL).
    virtual void render(cRenderOptions& a_options);
};


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
