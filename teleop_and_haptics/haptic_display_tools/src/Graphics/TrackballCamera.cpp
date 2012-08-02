#include "TrackballCamera.h"
#include <cml/cml.h>

using namespace std;
using namespace cml;

// --------------------------------------------------------------------------

TrackballCamera::TrackballCamera(vector3f origin, float radius)
    : OrbitingCamera(origin, radius, 0.1f * radius, 10.f * radius)
{
    m_trackballCenter = vector2f(100.f, 100.f);
    m_trackballRadius = 100.f;

    m_orientation.identity();
    m_rotation.identity();
}

// --------------------------------------------------------------------------

void TrackballCamera::orient(cml::vector3f view, cml::vector3f up)
{
    // we keep the orientation as the transpose of the camera basis vectors,
    // so the alignment matrix must be transposed to get the orientation
    matrix_rotation_align(m_orientation, -view, up);
    m_orientation.transpose();
    computeViewMatrix();
}

// --------------------------------------------------------------------------

vector3f TrackballCamera::trackballVector(int x, int y)
{
    // center and normalize the viewport point
    vector2f p(x, y);
    p -= m_trackballCenter;
    p /= m_trackballRadius;

    float lsquared = p.length_squared();

    // if we are grabbing outside the bounds of the virtual hemisphere,
    // take a point on the edge
    if (lsquared > 1.0) {
        p.normalize();
        return vector3f(p[0], p[1], 0.0f);
    }
    // otherwise we are on the protruding hemisphere
    else {
        float z = sqrtf(1.0f - lsquared);
        return vector3f(p[0], p[1], z);
    }
}

// --------------------------------------------------------------------------

void TrackballCamera::computeViewMatrix()
{
    // translation to origin
    matrix44f translate;
    matrix_translation(translate, -origin());

    // rotation and translation by radius
    matrix44f rotate = m_rotation * m_orientation;
    matrix_set_translation(rotate, 0.f, 0.f, -m_radius);

    m_view = rotate * translate;
}

// --------------------------------------------------------------------------

void TrackballCamera::resize(int width, int height)
{
    m_trackballCenter = 0.5f  * vector2f(width, height);
    m_trackballRadius = 0.25f * (abs(width) + abs(height));
}

// --------------------------------------------------------------------------

void TrackballCamera::mouseDown(int x, int y)
{
    // remember where the current rotation started
    m_start = trackballVector(x, y);
}

void TrackballCamera::mouseUp(int x, int y)
{
    // here we should commit the current rotation
    vector3f finish = trackballVector(x, y);
    matrix_rotation_vec_to_vec(m_rotation, m_start, finish, true);

    // compose the temporary rotation with the accumulated one
    m_orientation = m_rotation * m_orientation;
    m_rotation.identity();

    // update the view matrix
    computeViewMatrix();
}

void TrackballCamera::mouseMove(int x, int y)
{
    // compute the current rotation and update the view matrix
    vector3f finish = trackballVector(x, y);
    matrix_rotation_vec_to_vec(m_rotation, m_start, finish, true);
    computeViewMatrix();
}

// --------------------------------------------------------------------------
