#ifndef CPLANE_H
#define CPLANE_H

#include "chai3d.h"

class cPlane : public cMesh
{
public:
    cPlane(cWorld* a_world);

    cVector3d m_normal;

};

#endif // CPLANE_H
