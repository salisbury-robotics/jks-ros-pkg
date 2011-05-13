#ifndef SPHERICALCAMERA_H
#define SPHERICALCAMERA_H

#include "Camera.h"

class SphericalCamera : public OrbitingCamera
{
protected:
    float           m_azimuth, m_altitude;
    float           m_altitudeLimit;
    int             m_cardinalUp;   // 0 for x, 1 for y, 2 for z

public:
    SphericalCamera(cml::vector3f origin = cml::vector3f(0,0,0),
                    float radius = 2.f, int cardinalUp = 1);

    void            setAzimuth(float theta)         { m_azimuth = theta; }
    void            setAltitude(float phi)          { m_altitude = phi; }

    // mutators for setting camera parameters
    void            setAltitudeLimit(float limit)   { m_altitudeLimit = limit; }
    void            setCardinalUp(int up)           { m_cardinalUp = up; }

    // helper function to compute the view matrix from spherical parameters
    virtual void    computeViewMatrix();

    // call these functions when mouse input is captured
    virtual void    mouseMove(int x, int y);
};

#endif // SPHERICALCAMERA_H
