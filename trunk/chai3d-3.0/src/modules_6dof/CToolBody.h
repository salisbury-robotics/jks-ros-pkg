
//---------------------------------------------------------------------------
#ifndef ToolBodyH
#define ToolBodyH
//---------------------------------------------------------------------------
#include "tools/CGenericTool.h"
//---------------------------------------------------------------------------

namespace srl {

class ToolBody : public cGenericTool
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of ToolBody.
    ToolBody(cWorld* a_parentWorld);

    //! Destructor of ToolBody.
    virtual ~ToolBody();


    //-----------------------------------------------------------------------
    // MEMBERS
    //-----------------------------------------------------------------------
    
    // Single contact point of cursor.
    cInteractionPoint* m_interactionPoint;

    //cGenericObject m;

    //-----------------------------------------------------------------------
    // METHODS
    //-----------------------------------------------------------------------

    //! Compute interaction forces between cursor's contact point and environment.
    virtual void computeInteractionForces();

    //! Render the object in OpenGL.
    virtual void render(cRenderOptions& a_options);

    //! Update the position and orientation of the tool image.
    virtual void updateToolImagePosition();
};

} // namespace srl

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

