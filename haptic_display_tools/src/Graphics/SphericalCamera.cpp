#include "SphericalCamera.h"
#include <cml/cml.h>

using namespace std;
using namespace cml;

// --------------------------------------------------------------------------

SphericalCamera::SphericalCamera(vector3f origin, float radius, int cardinalUp)
    : OrbitingCamera(origin, radius, 0.1f * radius, 10.f * radius),
      m_azimuth(0), m_altitude(15)
{
    m_altitudeLimit = 75.f;
    m_cardinalUp    = cardinalUp;   // default Y-axis is up

    computeViewMatrix();
}

// --------------------------------------------------------------------------

void SphericalCamera::computeViewMatrix()
{
    // convert current spherical parameters to cartesian
    vector3f position;
    spherical_to_cartesian(radius(), rad(m_azimuth), rad(m_altitude),
                           m_cardinalUp, latitude, position);
    position += m_origin;

    // generate a transformation matrix for the camera
    matrix_look_at_RH(m_view, position, origin(), axis_3D(m_cardinalUp));
}

// --------------------------------------------------------------------------

void SphericalCamera::mouseMove(int x, int y)
{
    vector2i delta = vector2i(x,y) - m_mouseLast;

    // update azimuth (longitude) based on x movement
    m_azimuth += 0.2f * delta[0];
    m_azimuth = fmodf(m_azimuth+360.0f, 360.0f);

    // update altitude (latitude) based on y movement
    m_altitude += 0.2f * delta[1];
    m_altitude = clamp(m_altitude, -m_altitudeLimit, m_altitudeLimit);

    // compute the new view matrix
    computeViewMatrix();

    // call base class' mouse move
    OrbitingCamera::mouseMove(x, y);
}

// --------------------------------------------------------------------------
