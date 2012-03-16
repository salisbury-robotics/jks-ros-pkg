//---------------------------------------------------------------------------
#ifdef _MSVC
#pragma warning (disable : 4786)
#endif
//---------------------------------------------------------------------------
#ifndef CRIGIDBODY_H
#define CRIGIDBODY_H
//---------------------------------------------------------------------------
#include "world/CGenericObject.h"
#include "materials/CMaterial.h"
#include "materials/CTexture2d.h"
#include "graphics/CColor.h"
#include <vector>
#include <list>
//---------------------------------------------------------------------------
class cWorld;
class cMesh;
//class cTriangle;
//class cVertex;
//---------------------------------------------------------------------------

namespace srl {

class cRigidBody : public cGenericObject
{
public:
    cRigidBody();

    //-----------------------------------------------------------------------
    // MEMBERS - GEOMETRIC PROPERTIES:
    //-----------------------------------------------------------------------

    enum GEOMETRY_TYPE{
      TRIANGLE_MESH,
      TETRA_MESH,
      DISCRETE_GRID,
      SAMPLED_VOLUME,
    } m_geometryType;

    cMesh *m_mesh;
    //cGrid *m_grid;
    std::vector< cVector3d > m_pointShell;

    //-----------------------------------------------------------------------
    // MEMBERS - PHYSICAL PROPERTIES:
    //-----------------------------------------------------------------------

    //! The density of the body, assumed to be uniform.
    double m_density;

    //! The mass of the body.
    double m_mass;

    //! The body-aligned inertia matrix (dyadic) of the body.
    cMatrix3d m_inertia;

    //! Flag indicating an immovable object
    bool m_isFixed;

    //-----------------------------------------------------------------------
    // MEMBERS - DYNAMIC STATE:
    //-----------------------------------------------------------------------

    //! The linear velocity of the center of mass, relative to and expressed in world coordinates.
    cVector3d m_linVelocity;

    //! The angular velocity of this body, relative to and expressed in world coordinates.
    cVector3d m_angVelocity;

  };

} // namespace srl

#endif // CRIGIDBODY_H
