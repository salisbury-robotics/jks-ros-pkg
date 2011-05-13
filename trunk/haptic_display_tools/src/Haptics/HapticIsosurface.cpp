#include "HapticIsosurface.h"
#include <algorithm>

using namespace cml;

// --------------------------------------------------------------------------
// Visual C++ can't handle static const double initialization within the
// class declaration, so we put them here

const double HapticIsosurface::k_stepTangent = 0.01;
const double HapticIsosurface::k_stepNormal = 0.005;

// --------------------------------------------------------------------------

HapticIsosurface::HapticIsosurface(Volume *volume, Volume *mask, double isoValue)
    : m_sampler(0), m_isoValue(isoValue)
{
    if (volume) m_sampler = new VolumeSampler(volume, mask);

    // create a volume cutter on the mask to remove material
    if (mask)   m_maskCutter = new VolumeCutter(mask);
}

HapticIsosurface::~HapticIsosurface()
{
    if (m_sampler) delete m_sampler;
    if (m_maskCutter) delete m_maskCutter;
}


void HapticIsosurface::setIsosurfaceValue(double value)
{
    m_isoValue = value;
    reset();
}

void HapticIsosurface::reset()
{
    // clear out the ready states so that the proxy algorithm is reset
    for (int i = 0; i < k_maxStates; ++i)
        m_states[i].ready = m_states[i].inside = false;
}

// --------------------------------------------------------------------------
// This algorithm is from Salisbury & Tarr's 1997 paper.  It will find the
// closest point on the surface starting from a seed point that is close by
// following the direction of the field gradient.

vector3d HapticIsosurface::findSurface(const cml::vector3d &seed)
{
    const double epsilon = 1e-10;
    const int iterations = 10;

    vector3d p = seed, dp;
    for (int i = 0; i < iterations; ++i)
    {
        double s = surface(p);
        vector3d gs = gradient(p);
        dp = (-s * gs) / std::max(gs.length_squared(), epsilon);
        p = p + dp;
        if (dp.length_squared() < epsilon) break;
    }

    return p;
}

// --------------------------------------------------------------------------
// This algorithm tries to find the surface given a nearby seed point and a
// direction to search.  It uses interval bisection for a fixed number of
// iterations, and will not move the point farther than the magnitude of the
// provided direction vector.

vector3d HapticIsosurface::findSurface(const vector3d &seed,
                                       const vector3d &direction)
{
    const int iterations = 8;

    vector3d p = seed;
    vector3d d = direction;
    double a, b;
    a = b = surface(p);
    for (int i = 0; i < iterations; ++i)
    {
        d *= 0.5;
        if (a < 0.0)        p -= d;
        else if (a > 0.0)   p += d;
        else                break;
        a = surface(p);
    }

    // check if there is an improvement, and return original point if not
    return (fabs(a) < fabs(b)) ? p : seed;
}

// --------------------------------------------------------------------------
// The update method basically implements the implicit surface rendering
// technique described in Salisbury & Tarr 1997.

void HapticIsosurface::update(HapticDisplay *display)
{
    // bail out if we run out of states
    if (display->identifier() >= k_maxStates) return;

    // fetch the state for the correct haptic display
    State &s = m_states[display->identifier()];

    // let q be the new device position
    vector3d q = display->toolPosition();
    vector3d &p = s.proxy;

    // do a check to ensure the algorithm does not start inside the object
    if (!s.ready) {
        if (surface(q) < 0.0) {
            display->setProxyPosition(q);
            return;
        }
        else s.ready = true;
    }

    // if we're starting inside...
    if (s.inside)
    {
        // get the tangent plane from the last projected point
        vector3d n = gradient(p);

        // test if we've broken contact
        if (dot(q-p, n) > 0.0) {
            p = q;
            s.inside = false;
        }
        // track point on surface
        else {
            // first project q onto the tangent plane
            if (n.length_squared() != 0.0) n.normalize();
            vector3d v = q-p;
            vector3d t = q - dot(v, n) * n;

            // limit the tangent step if needed
            if (length_squared(t-p) > k_stepTangent * k_stepTangent) {
                vector3d step = normalize(t-p) * k_stepTangent;
                t = p + step;
            }

            // check for and mitigate limit cycle on concave surface
            if (surface(t) < 0.0) t = 0.5 * (t + p);

            // limit the normal step if needed
            if (v.length_squared() > k_stepNormal * k_stepNormal) {
                v.normalize();
                v *= k_stepNormal;
            }

            p = findSurface(t, v);
        }
    }
    // otherwise detect first collision
    else
    {
        if (surface(q) < 0.0) {
            p = findSurface(p, q-p);
            s.inside = true;
        }
        else p = q;
    }

    display->setProxyPosition(p);
    display->setProxyOrientation(display->toolOrientation());
}

// --------------------------------------------------------------------------
