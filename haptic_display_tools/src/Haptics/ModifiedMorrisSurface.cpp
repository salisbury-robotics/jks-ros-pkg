#include "ModifiedMorrisSurface.h"
#include <cml/cml.h>

using namespace cml;

// --------------------------------------------------------------------------

ModifiedMorrisSurface::ModifiedMorrisSurface(Sampler *sampler,
                                             double isoValue, double radius)
    : HapticIsosurface(sampler, isoValue)
{
    // create a new sampler for the labels
    m_labelSampler = new VolumeSampler();

    setToolRadius(radius);
}

// --------------------------------------------------------------------------

void ModifiedMorrisSurface::createSampleSphere(double r, int n)
{
    // if numer of samples not give, use the constant k_samples
    if (n < 0) n = k_samples;

    // generate uniform random vectors until we reach the quota
    for (int c = 0; c < n; )
    {
        vector3d p(random_real(-1.0, 1.0),
                   random_real(-1.0, 1.0),
                   random_real(-1.0, 1.0));
        p *= r;
        double l2 = p.length_squared();
        if (l2 < r*r && l2 > 0.2*r*r) {
            m_samples[c] = p;
            m_directions[c] = -normalize(p);
            ++c;
        }
    }
}

// --------------------------------------------------------------------------

double ModifiedMorrisSurface::pointForceMagnitude(HapticDisplay *display)
{
    // first get the tool tip radius in physical units (m)
    double r = display->toolRadius() * display->physicalToWorldScale();

    // then use the radius desired stiffness to get a point force value
    return r * display->stiffness() * 5.0 / k_samples;
}

// --------------------------------------------------------------------------

void ModifiedMorrisSurface::update(HapticDisplay *display)
{
    // use the original Haptic Isosurface method to update proxy
    HapticIsosurface::update(display);

    // fetch the state for the correct haptic display
    State &s = m_states[display->identifier()];

    // then compute addition force from sample interference
    if (s.ready)
    {
        // compute the magnitude of force each point should contribute
        double pfm = pointForceMagnitude(display);

        // compute force as described by Morris et al. 2006 using sphere
        // centered at the proxy position
        vector3d p = display->proxyPosition();
        vector3d f = zero_3D();
        for (int i = 0; i < k_samples; ++i)
        {
            // feed back forces for the label volume too...
            if (surface(p + m_samples[i]) < 0.0 ||
                m_labelSampler->intensityAt(p + m_samples[i]) >= 0.5)
            {
                f += m_directions[i] * pfm;
            }
        }
        display->setDirectForce(f);

        // mill away the mask if the first button on the device is down
        // TODO
//        if (display->buttonState(0))
//        {
//            m_maskCutter->cutFilteredSphere(p, 0.8 * m_toolRadius);
//        }
    }
}

// --------------------------------------------------------------------------
