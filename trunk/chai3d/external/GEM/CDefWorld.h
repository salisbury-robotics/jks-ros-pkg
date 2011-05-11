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
#ifndef CDefWorldH
#define CDefWorldH
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CDefMesh.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
      \class      cDefWorld
      \brief      cDefWorld implements a world to handle deformable objects
                  within CHAI 3D.
*/
//===========================================================================
class cDefWorld : public cGenericObject
{
  public:
    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cDefWorld
    cDefWorld();

    //! Destructor of cDefWorld
    ~cDefWorld();

    // PROPERTIES:
    //! List of deformable solids.
    list<cDefMesh*> m_defMeshes;

    //! current time of simulation
    double m_simulationTime;

    //! integration time
    double m_integrationTime;

    //! gravity constant
    cVector3d m_gravity;

    // METHODS:
    //! compute dynamic simulation
    void updateDynamics(double a_time);

    //! clear external forces on all objects
    void clearExternalForces();

    // update vertices of all objects
    void updateSkins();

  private:
    // render deformable mesh
    virtual void render(const int a_renderMode=CHAI_RENDER_MODE_RENDER_ALL);
};


//===========================================================================
/*!
      \class      cDefWorldCollision
      \brief      cDefWorldCollision provides a collision detection model
                  to support deformable objects
*/
//===========================================================================
class cDefWorldCollision : public cGenericCollision
{
  public:
    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cDefWorldCollision
    cDefWorldCollision(cDefWorld* a_defWorld) {m_defWorld = a_defWorld;}

    //! Destructor of cDefWorldCollision
    virtual ~cDefWorldCollision() {};

    // VIRTUAL METHODS:
    //! Do any necessary initialization, such as building trees.
    virtual void initialize() {};

    //! Provide a visual representation of the method.
    virtual void render() {};

    //! Return the nearest triangle intersected by the given segment, if any.
    virtual bool computeCollision(cVector3d& a_segmentPointA,
                                  cVector3d& a_segmentPointB,
                                  cCollisionRecorder& a_recorder,
                                  cCollisionSettings& a_settings);

  private:
    //! deformable world
    cDefWorld* m_defWorld;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------


