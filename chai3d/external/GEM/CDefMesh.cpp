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
#include "CDefMesh.h"
//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
#include <cfloat>

//===========================================================================
/*!
     Initialise deformable mesh

      \fn       void cDefMesh::initialise()
*/
//===========================================================================
void cDefMesh::initialise()
{
    m_showSkeleton = true;
}


//===========================================================================
/*!
     Clear forces

      \fn       void cDefMesh::clearForces()
*/
//===========================================================================
void cDefMesh::clearForces()
{
    list<cDefNode*>::iterator i;

    for(i = m_nodes.begin(); i != m_nodes.end(); ++i)
    {
        cDefNode* nextItem = *i;
        nextItem->clearForces();
    }
}


//===========================================================================
/*!
     Clear external forces on nodes

      \fn       void cDefMesh::clearExternalForces()
*/
//===========================================================================
void cDefMesh::clearExternalForces()
{
    list<cDefNode*>::iterator i;

    for(i = m_nodes.begin(); i != m_nodes.end(); ++i)
    {
        cDefNode* nextItem = *i;
        nextItem->clearExternalForces();
    }
}


//===========================================================================
/*!
     Compute all internal forces

      \fn       void cDefMesh::computeForces()
*/
//===========================================================================
void cDefMesh::computeForces()
{
    list<cDefLink*>::iterator i;

    for(i = m_links.begin(); i != m_links.end(); ++i)
    {
        cDefLink* nextItem = *i;
        nextItem->computeForces();
    }
}

//===========================================================================
/*!
     Compute next pose of each node.

      \fn       void cDefMesh::computeNextPose(double a_timeInterval)
*/
//===========================================================================
void cDefMesh::computeNextPose(double a_timeInterval)
{
    list<cDefNode*>::iterator i;

    for(i = m_nodes.begin(); i != m_nodes.end(); ++i)
    {
        cDefNode* nextItem = *i;
        nextItem->computeNextPose(a_timeInterval);
    }
}


//===========================================================================
/*!
     Apply the next pose of each node.

      \fn       void cDefMesh::applyNextPose()
*/
//===========================================================================
void cDefMesh::applyNextPose()
{
    list<cDefNode*>::iterator i;

    for(i = m_nodes.begin(); i != m_nodes.end(); ++i)
    {
        cDefNode* nextItem = *i;
        nextItem->applyNextPose();
    }
}


//===========================================================================
/*!
     Render this deformable mesh in OpenGL.

     \fn       void cDefMesh::render(const int a_renderMode)
     \param    a_renderMode  Rendering mode (see cGenericObject)
*/
//===========================================================================
void cDefMesh::render(const int a_renderMode)
{
    /////////////////////////////////////////////////////////////////////////
    // Conditions for object to be rendered
    /////////////////////////////////////////////////////////////////////////

    if((a_renderMode == CHAI_RENDER_MODE_TRANSPARENT_FRONT_ONLY) ||
       (a_renderMode == CHAI_RENDER_MODE_TRANSPARENT_BACK_ONLY))
    {
        return;
    }

    /////////////////////////////////////////////////////////////////////////
    // Rendering code here
    /////////////////////////////////////////////////////////////////////////
    list<cDefNode*>::iterator i;
    if (m_showSkeleton)
    {
        for(i = m_nodes.begin(); i != m_nodes.end(); ++i)
        {
            cDefNode* nextItem = *i;
            nextItem->render();
        }

        list<cDefLink*>::iterator j;

        for(j = m_links.begin(); j != m_links.end(); ++j)
        {
            cDefLink* nextItem = *j;
            nextItem->render();
        }
    }
}


//===========================================================================
/*!
     Connect each vertex to nearest node.

     \fn       void cDefMesh::connectVerticesToSkeleton(bool a_connectToNodesOnly)
     \a_connectToNodesOnly  if TRUE, then skin is only connected to nodes.
                otherwise skein shall be connected to links too.
*/
//===========================================================================
void cDefMesh::connectVerticesToSkeleton(bool a_connectToNodesOnly)
{
    // clear all deformable vertices
    m_defVertices.clear();

    // get number of vertices
    int numVertices = getNumVertices(true);

    // create a table of deformable vertices based on the vertices of image
    for (int i=0; i<numVertices; i++)
    {
        // create a new deformable vertex data
        cDefVertex newDefVertex;
        newDefVertex.vertex = getVertex(i, true);
        newDefVertex.link = NULL;
        newDefVertex.node = NULL;
        newDefVertex.pos.zero();

        // add vertex to list
        m_defVertices.push_back(newDefVertex);
    }

    // for each deformable vertex we search for the nearest sphere or link
    for (int i=0; i<numVertices; i++)
    {
        // get current deformable vertex
        cDefVertex* curVertex = &m_defVertices[i];

        // get current vertex position
        cVector3d pos = curVertex->vertex->getPos();

        // initialize constant
        double min_distance = DBL_MAX;  // std::numeric_limits<double>::max();
        cDefNode* nearest_node = NULL;
        cDefLink* nearest_link = NULL;

        // search for the nearest node
        for(list<cDefNode*>::iterator i = m_nodes.begin(); i != m_nodes.end(); ++i)
        {
            cDefNode* nextNode = *i;
            double distance = cDistance(pos, nextNode->m_pos);
            if (distance < min_distance)
            {
                min_distance = distance;
                nearest_node = nextNode;
                nearest_link = NULL;
            }
        }

        // search for the nearest link if any
        if (!a_connectToNodesOnly)
        {
            list<cDefLink*>::iterator j;
            for(j = m_links.begin(); j != m_links.end(); ++j)
            {
                cDefLink* nextLink = *j;
                double angle0 = cAngle(nextLink->m_wLink01, cSub(pos, nextLink->m_node0->m_pos));
                double angle1 = cAngle(nextLink->m_wLink10, cSub(pos, nextLink->m_node1->m_pos));

                if ((angle0 < (CHAI_PI / 2.0)) && (angle1 < (CHAI_PI / 2.0)))
                {
                    cVector3d p = cProjectPointOnLine(pos,
                                                      nextLink->m_node0->m_pos,
                                                      nextLink->m_wLink01);

                    double distance = cDistance(pos, p);

                    if (distance < min_distance)
                    {
                        min_distance = distance;
                        nearest_node = NULL;
                        nearest_link = nextLink;
                    }
                }
            }
        }

        // attach vertex to nearest node if it exists
        if (nearest_node != NULL)
        {
            curVertex->node = nearest_node;
            curVertex->link = NULL;
            cVector3d posRel = cSub(pos, nearest_node->m_pos);
            curVertex->pos = cMul(cTrans(nearest_node->m_rot), posRel);
        }

        // attach vertex to nearest link if it exists
        else if (nearest_link != NULL)
        {
            curVertex->node = NULL;
            curVertex->link = nearest_link;

            cMatrix3d rot;
            rot.setCol( nearest_link->m_A0,
                        nearest_link->m_B0,
                        nearest_link->m_wLink01);
            cVector3d posRel = cSub(pos, nearest_link->m_node0->m_pos);
            curVertex->pos = cMul(cInv(rot), posRel);
        }
    }
}


//===========================================================================
/*!
     Update position of vertices connected to skeleton.

     \fn       void cDefMesh::updateVertexPosition()
*/
//===========================================================================
void cDefMesh::updateVertexPosition()
{
   // get number of vertices
    int numVertices = getNumVertices(true);

    // for each deformable vertex, update its position
    for (int i=0; i<numVertices; i++)
    {
        // get current deformable vertex
        cDefVertex* curVertex = &m_defVertices[i];

        // the vertex is attached to an node
        if (curVertex->node != NULL)
        {
            cVector3d newPos;
            curVertex->node->m_rot.mulr(curVertex->pos, newPos);
            newPos.add(curVertex->node->m_pos);
            curVertex->vertex->setPos(newPos);
        }

        else if (curVertex->link != NULL)
        {
            cVector3d newPos;
            curVertex->link->m_node0->m_pos.addr(curVertex->pos.z * curVertex->link->m_wLink01, newPos);
            newPos.add(curVertex->pos.x * curVertex->link->m_wA0);
            newPos.add(curVertex->pos.y * curVertex->link->m_wB0);


            //curVertex->node->m_rot.mulr(curVertex->pos, newPos);
            //newPos.add(curVertex->node->m_pos);
            curVertex->vertex->setPos(newPos);
        }
    }
}


