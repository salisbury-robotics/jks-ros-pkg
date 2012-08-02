#ifndef TRACKBALLCAMERA_H
#define TRACKBALLCAMERA_H

#include "Graphics/Camera.h"

class TrackballCamera : public OrbitingCamera
{
protected:
    // center and radius of the virtual trackball, in viewport coordinates
    cml::vector2f   m_trackballCenter;
    float           m_trackballRadius;

    // rotation-only matrix that goes from world to camera frame
    //  (transpose basis vectors are camera's frame vectors)
    cml::matrix44f  m_orientation;

    // variables that track a temporary state as the mouse is dragged
    cml::vector3f   m_start;
    cml::matrix44f  m_rotation;

    // converts viewport coordinates to 3D vector on the trackball
    cml::vector3f   trackballVector(int x, int y);

public:
    TrackballCamera(cml::vector3f origin = cml::vector3f(0,0,0),
                    float radius = 2.f);

    // set the intial view direction of the camera
    void            orient(cml::vector3f view, cml::vector3f up);

    virtual void    computeViewMatrix();

    // the trackball camera inverts the movement to make it feel as if we
    // are manipulating the object rather than the camera
    virtual void    move(float truck, float pedestal, float dolly)
                        { OrbitingCamera::move(-truck, -pedestal, -dolly); }

    // set width and height of the viewport, so that a trackball can be placed
    //  (note: negative height can be used if Y-axis is inverted)
    virtual void    resize(int width, int height);

    // call these functions when mouse input is captured
    virtual void    mouseDown(int x, int y);
    virtual void    mouseUp(int x, int y);
    virtual void    mouseMove(int x, int y);

};

#endif // TRACKBALLCAMERA_H
