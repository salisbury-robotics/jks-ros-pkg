#ifndef PROXYGEOMETRY_H
#define PROXYGEOMETRY_H

#include <cml/mathlib/typedef.h>

class ProxyGeometry
{
    // lower and upper corners of the proxy box
    cml::vector3f m_lower, m_upper;

public:
    ProxyGeometry(const cml::vector3f &lower = cml::vector3f(0.f, 0.f, 0.f),
                  const cml::vector3f &upper = cml::vector3f(1.f, 1.f, 1.f));

    // issues the requisite OpenGL calls to render the proxy geometry
    void render();
};

#endif // PROXYGEOMETRY_H
