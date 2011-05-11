//===========================================================================
/*
    This file is part of the GEM dynamics engine.
    Copyright (C) 2003-#YEAR# by Francois Conti, Stanford University.
    All rights reserved.

    \author:    <http://www.chai3d.org>
    \author:    Francois Conti
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "CDefWorld.h"
//---------------------------------------------------------------------------

//==========================================================================
/*!
      Extends cDynWorld to support collision detection

      \fn       cDefWorld::cDefWorld()
      \return   Return pointer to new cDefWorld instance.
*/
//===========================================================================
bool cDefWorldCollision::computeCollision(cVector3d& a_segmentPointA,
                                          cVector3d& a_segmentPointB,
                                          cCollisionRecorder& a_recorder,
                                          cCollisionSettings& a_settings)
{
    list<cDefMesh*>::iterator i;
    bool result = false;
    for(i = m_defWorld->m_defMeshes.begin(); i != m_defWorld->m_defMeshes.end(); ++i)
    {
        cDefMesh *nextItem = *i;
        bool collide = nextItem->computeCollisionDetection(a_segmentPointA, a_segmentPointB, a_recorder,
                        a_settings);
        if (collide) { result = true; }
    }
    return (result);
}


//==========================================================================
/*!
      Constructor of cDefWorld.

      \fn       cDefWorld::cDefWorld()
      \return   Return pointer to new cDefWorld instance.
*/
//===========================================================================
cDefWorld::cDefWorld()
{
    // reset simulation time.
    m_simulationTime = 0.0;

    // set a default value for the integration time step [s].
    m_integrationTime = 1.0f / 400.0f;

    // create a collision detector for world
    m_collisionDetector = new cDefWorldCollision(this);
}


//===========================================================================
/*!
      Destructor of cDefWorld

      \fn       cDefWorld::~cDefWorld()
*/
//===========================================================================
cDefWorld::~cDefWorld()
{
    m_defMeshes.clear();
}


//===========================================================================
/*!
     Render world in OpenGL.

     \fn       void cDefWorld::render(const int a_renderMode)
     \param    a_renderMode  Rendering mode (see cGenericObject)
*/
//===========================================================================
void cDefWorld::render(const int a_renderMode)
{
    list<cDefMesh*>::iterator i;

    for(i = m_defMeshes.begin(); i != m_defMeshes.end(); ++i)
    {
        cDefMesh* nextItem = *i;
        nextItem->renderSceneGraph(a_renderMode);
    }
}

//===========================================================================
/*!
      Clear external forces on all deformable objects in scene.

      \fn       void cDefWorld::clearExternalForces()
*/
//===========================================================================
void cDefWorld::clearExternalForces()
{
    list<cDefMesh*>::iterator i;

    for(i = m_defMeshes.begin(); i != m_defMeshes.end(); ++i)
    {
        cDefMesh *nextItem = *i;
        nextItem->clearExternalForces();
    }
}


//===========================================================================
/*!
      Compute simulation for a_time time interval.

      \fn       void cDefWorld::updateDynamics(double a_time)
*/
//===========================================================================
void cDefWorld::updateDynamics(double a_time)
{
    list<cDefMesh*>::iterator i;

    double nextTime = m_simulationTime + a_time;

    while (m_simulationTime < nextTime)
    {
        // clear all internal forces of each model
        for(i = m_defMeshes.begin(); i != m_defMeshes.end(); ++i)
        {
            cDefMesh *nextItem = *i;
            nextItem->clearForces();
        }

        // compute all internal forces for ach model
        for(i = m_defMeshes.begin(); i != m_defMeshes.end(); ++i)
        {
            cDefMesh *nextItem = *i;
            nextItem->computeForces();
        }

        // compute next pose of model
        for(i = m_defMeshes.begin(); i != m_defMeshes.end(); ++i)
        {
            cDefMesh *nextItem = *i;
            nextItem->computeNextPose(m_integrationTime);
        }

        // apply next pose
        for(i = m_defMeshes.begin(); i != m_defMeshes.end(); ++i)
        {
            cDefMesh *nextItem = *i;
            nextItem->applyNextPose();
        }

        // update simulation time
        m_simulationTime = m_simulationTime + m_integrationTime;
    }
}

//===========================================================================
/*!
      Update vertices of all objects

      \fn       void cDefWorld::updateDynamics(double a_time)
*/
//===========================================================================
void cDefWorld::updateSkins()
{
    // update surface mesh to latest skelton configuration
    list<cDefMesh*>::iterator i;
    for(i = m_defMeshes.begin(); i != m_defMeshes.end(); ++i)
    {
        cDefMesh *nextItem = *i;
        nextItem->updateVertexPosition();
    }
}

