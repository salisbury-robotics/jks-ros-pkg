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
#ifndef CDefMeshH
#define CDefMeshH
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CDefNode.h"
#include "CDefMesh.h"
#include "CDefNode.h"
#include "CDefLink.h"
#include <typeinfo>
#include <vector>
#include <list>
//---------------------------------------------------------------------------
using std::vector;
//---------------------------------------------------------------------------

struct cDefVertex
{
    cVertex*    vertex;
    cDefLink*   link;
    cDefNode*   node;
    cVector3d   pos;
};

//===========================================================================
/*!
      \class      cDefMesh
      \brief      cDefMesh inherits from cmesh and integrate a skelton model
                  for deformation simulation
*/
//===========================================================================
class cDefMesh : public cMesh
{

  public:
    // CONSTRUCTOR & DESTRUCTOR:

    //! Constructor of cMesh.
    cDefMesh(cWorld* a_world):cMesh(a_world){initialise();};

    //! Destructor of cMesh.
    virtual ~cDefMesh() {};

    // METHODS:
    //! Connect each vertex to skeleton.
    void connectVerticesToSkeleton(bool a_connectToNodesOnly);

    //! Update position of vertices connected to skeleton.
    void updateVertexPosition();

    //! clear forces
    void clearForces();

    //! clear external forces
    void clearExternalForces();

    //! compute forces
    void computeForces();

    //! compute next pose
    void computeNextPose(double iTimeInterval);

    //! apply new computed pose
    void applyNextPose();

    // MEMBERS:
    //! List of nodes composing the skeleton.
    list<cDefNode*> m_nodes;

    //! List of links connecting the different nodes.
    list<cDefLink*> m_links;

    //! If TRUE then display skeleton
    bool m_showSkeleton;

    // render deformable mesh
    virtual void render(const int a_renderMode=CHAI_RENDER_MODE_RENDER_ALL);


  private:
    // list of deformable vertices
    vector<cDefVertex> m_defVertices;

    // initialise deformable mesh
    void initialise();
};

#endif
