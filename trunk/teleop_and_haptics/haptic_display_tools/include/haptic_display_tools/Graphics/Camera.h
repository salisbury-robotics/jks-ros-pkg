#ifndef CAMERA_H
#define CAMERA_H

#include <cml/mathlib/typedef.h>
#include <cml/mathlib/vector_transform.h>

// --------------------------------------------------------------------------

class Camera
{
protected:
    cml::matrix44f_c    m_view;         // current view matrix
    cml::vector2i       m_mouseLast;    // last received mouse coordinates

public:
    Camera()            { m_view.identity(); m_mouseLast = cml::vector2i(0,0); }
    virtual ~Camera()   { }

    // insert the view matrix into the OpenGL matrix stack to pose the camera
    cml::matrix44f_c    viewMatrix() const      { return m_view; }

    // implementation decides whether or not this needs to be called explicityly
    virtual void        computeViewMatrix()     { }

    // moves the camera in its local frame
    virtual void        move(float truck, float pedestal, float dolly) { }

    // call these functions when mouse input is captured
    virtual void        mouseDown(int x, int y) { m_mouseLast = cml::vector2i(x, y); }
    virtual void        mouseUp(int x, int y)   { m_mouseLast = cml::vector2i(x, y); }
    virtual void        mouseMove(int x, int y) { m_mouseLast = cml::vector2i(x, y); }
    virtual void        mouseScroll(int delta)  { }

    // accessor to retrieve the last accessed mouse coordinates
    cml::vector2i       mouseLast() const       { return m_mouseLast; }
};

// --------------------------------------------------------------------------
// a camera type that orbits around some point in space

class OrbitingCamera : public Camera
{
protected:
    cml::vector3f   m_origin;
    float           m_radius;
    std::pair<float, float> m_range;
    
public:
    OrbitingCamera(cml::vector3f origin = cml::vector3f(0.f, 0.f, 0.f),
                   float radius = 2.f, float rmin = 0.1f, float rmax = 10.f)
        : m_origin(origin), m_radius(radius), m_range(rmin, rmax) { }

    // accessor/mutator for camera origin
    cml::vector3f   origin() const                  { return m_origin; }
    void            setOrigin(cml::vector3f origin) { m_origin = origin; }

    // accessor/mutator for camera radius
    float           radius() const                  { return m_radius; }
    void            setRadius(float radius)
        { m_radius = cml::clamp(radius, m_range.first, m_range.second); }

    // mutator for camera radius min/max (range)
    void            setRange(float rmin, float rmax) {
                        m_range.first = rmin;
                        m_range.second = rmax;
                        setRadius(m_radius);
                    }

    // default implementation of mouse scroll to zoom (delta in degrees)
    virtual void    mouseScroll(int delta) {
                        setRadius(m_radius * expf(-delta / 360.f));
                        computeViewMatrix();
                    }

    // translates the camera's origin along the horizontal/vertical/optical axes
    virtual void    move(float truck, float pedestal, float dolly) {
                        cml::vector3f local(truck, pedestal, dolly);
                        m_origin += cml::transform_point(transpose(m_view), local*m_radius);
                        computeViewMatrix();
                    }
};

// --------------------------------------------------------------------------
#endif // CAMERA_H
