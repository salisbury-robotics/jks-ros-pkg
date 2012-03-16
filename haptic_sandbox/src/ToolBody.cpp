

//---------------------------------------------------------------------------
#include "modules_6dof/CToolBody.h"
#include "graphics/CTriangle.h"
//---------------------------------------------------------------------------

namespace srl {

//==========================================================================
/*!
    Constructor of ToolBody.

    \fn       ToolBody::ToolBody(cWorld* a_parentWorld):cGenericTool(a_parentWorld)
    \param    a_world  World in which the tool will operate.
*/
//===========================================================================
ToolBody::ToolBody(cWorld* a_parentWorld):cGenericTool(a_parentWorld)
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
    Destructor of ToolBody.

    \fn       ToolBody::~ToolBody()
*/
//===========================================================================
ToolBody::~ToolBody()
{
    delete m_interactionPoint;
}


//===========================================================================
/*!
    Update the position and orientation of the tool image.

    \fn       void ToolBody::computeInteractionForces()
*/
//===========================================================================
void ToolBody::updateToolImagePosition()
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

    \fn       void ToolBody::computeInteractionForces()
*/
//===========================================================================
void ToolBody::computeInteractionForces()
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

    \fn       void ToolBody::render(cRenderOptions& a_options)
    \param    a_options  Rendering options.
*/
//===========================================================================
void ToolBody::render(cRenderOptions& a_options)
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

} // namespace srl




