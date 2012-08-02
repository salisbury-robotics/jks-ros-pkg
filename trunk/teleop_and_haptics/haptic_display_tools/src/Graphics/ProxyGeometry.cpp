#include "ProxyGeometry.h"
#include "OpenGL.h"
#include <cml/cml.h>

using namespace cml;

ProxyGeometry::ProxyGeometry(const vector3f &lower, const vector3f &upper)
    : m_lower(lower), m_upper(upper)
{
}

void ProxyGeometry::render()
{
    // unit cube from Jim Blinn's Corner, Platonic Solids
    // IEEE Computer Graphics & Applications, 1987:7(11)
    static const vector3f vc[8] = {
        vector3f( 1.f, 1.f, 1.f ), vector3f( 1.f, 1.f,-1.f ),
        vector3f( 1.f,-1.f, 1.f ), vector3f( 1.f,-1.f,-1.f ),
        vector3f(-1.f, 1.f, 1.f ), vector3f(-1.f, 1.f,-1.f ),
        vector3f(-1.f,-1.f, 1.f ), vector3f(-1.f,-1.f,-1.f )
    };
    static const vector3f tc[8] = {
        vector3f( 1.f, 1.f, 1.f ), vector3f( 1.f, 1.f, 0.f ),
        vector3f( 1.f, 0.f, 1.f ), vector3f( 1.f, 0.f, 0.f ),
        vector3f( 0.f, 1.f, 1.f ), vector3f( 0.f, 1.f, 0.f ),
        vector3f( 0.f, 0.f, 1.f ), vector3f( 0.f, 0.f, 0.f )
    };

    static const int faces[6][4] = {
        { 2, 1, 3, 4 }, { 5, 6, 8, 7 }, { 1, 2, 6, 5 },
        { 4, 3, 7, 8 }, { 3, 1, 5, 7 }, { 2, 4, 8, 6 }
    };
    static vector3f normals[6] = { zero_3D() };

    // compute face normals once
    if (normals[0] == zero_3D()) {
        for (int f = 0; f < 6; ++f) {
            vector3f v = vc[faces[f][1]-1] - vc[faces[f][0]-1];
            vector3f w = vc[faces[f][3]-1] - vc[faces[f][0]-1];
            normals[f] = unit_cross(v, w);
        }
    }

    // this code issues texture coordinates the same as vertex coordinates
    matrix33f scale;
    matrix_scale(scale, m_upper - m_lower);
    glBegin(GL_QUADS);
        for (int i = 0; i < 6; ++i) {
            glNormal3fv(normals[i].data());
            for (int j = 0; j < 4; ++j) {
                vector3f p = scale * tc[faces[i][j]-1] + m_lower;
                glTexCoord3fv(p.data());
                glVertex3fv(p.data());
            }
        }
    glEnd();
}
