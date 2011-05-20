#ifndef MATERIAL_H
#define MATERIAL_H

#include "OpenGL.h"
#include <istream>
#include <cml/mathlib/typedef.h>

// --------------------------------------------------------------------------

typedef cml::vector4f colour;

// a modulate operator
inline colour operator*(const colour &a, const colour &b)
{
    return colour(a[0]*b[0], a[1]*b[1], a[2]*b[2], a[3]*b[3]);
}

// sets the opacity of a given colour
inline colour opaque(const colour &c, float alpha = 1.f)
{
    return colour(c[0], c[1], c[2], alpha);
}

// cml is missing a stream extraction operator, so we define one for colour
inline std::istream &operator>>(std::istream &stream, colour &c)
{
    return stream >> c[0] >> c[1] >> c[2] >> c[3];
}

const colour k_black(0.f, 0.f, 0.f, 1.f);
const colour k_white(1.f, 1.f, 1.f, 1.f);
const colour k_red  (1.f, 0.f, 0.f, 1.f);
const colour k_green(0.f, 1.f, 0.f, 1.f);
const colour k_blue (0.f, 0.f, 1.f, 1.f);

inline colour k_grey(float intensity = .5f) {
    return colour(intensity, intensity, intensity, 1.f);
}

// --------------------------------------------------------------------------

struct Light
{
    cml::vector4f   position;
    colour          ambient, diffuse, specular;

    Light(const cml::vector4f &p = cml::vector4f(0.f, 0.f, 0.f, 1.f),
          const colour &a = k_black, const colour &d = k_white,
          const colour &s = k_white)
          : position(p), ambient(a), diffuse(d), specular(s)
    { }

    void apply(GLenum light = GL_LIGHT0) const
    {
        glLightfv(light, GL_POSITION, position.data());
        glLightfv(light, GL_AMBIENT, ambient.data());
        glLightfv(light, GL_DIFFUSE, diffuse.data());
        glLightfv(light, GL_SPECULAR, specular.data());
    }
};

// --------------------------------------------------------------------------

struct Material
{
    colour  ambient, diffuse, specular, emission;
    float   shininess;

    Material(const colour &a = k_grey(.2f), const colour &d = k_grey(.8f),
             const colour &s = k_black, float k = 50.f,
             const colour &e = k_black)
          : ambient(a), diffuse(d), specular(s), emission(e), shininess(k)
    { }

    void apply(GLenum face = GL_FRONT) const
    {
        glMaterialfv(face, GL_AMBIENT, ambient.data());
        glMaterialfv(face, GL_DIFFUSE, diffuse.data());
        glMaterialfv(face, GL_SPECULAR, specular.data());
        glMaterialfv(face, GL_SHININESS, &shininess);
        glMaterialfv(face, GL_EMISSION, emission.data());
    }
};

// --------------------------------------------------------------------------
#endif // MATERIAL_H
