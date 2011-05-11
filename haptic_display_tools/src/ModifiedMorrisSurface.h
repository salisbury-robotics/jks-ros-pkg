#ifndef MODIFIEDMORRISSURFACE_H
#define MODIFIEDMORRISSURFACE_H

#include "HapticIsosurface.h"
#include <cml/mathlib/typedef.h>

class ModifiedMorrisSurface : public HapticIsosurface
{
protected:
    static const int    k_samples = 100;

    cml::vector3d       m_samples[k_samples];
    cml::vector3d       m_directions[k_samples];


    // additional sampler for a label volume
    VolumeSampler      *m_labelSampler;

    // creates n sample points that fill a sphere, with directions as unit
    // vectors pointing to the center (as described in Morris et al. 2006)
    void createSampleSphere(double r, int n = -1);

    // a haptic display carries with it all the information necessary to
    // compute the magntidue of force each sample point should carry
    double pointForceMagnitude(HapticDisplay *display);

public:
    ModifiedMorrisSurface(Volume *volume, Volume *mask = 0,
                          double isoValue = 0.5f, double radius = 0.02);

    // add a label volume for additional force feedback
    void setLabelVolume(Volume *v)      { m_labelSampler->setVolume(v); }

    // set the radius, in world units, of the spherical tool tip
    virtual void setToolRadius(double r) {
        HapticIsosurface::setToolRadius(r);
        createSampleSphere(r);
    }

    virtual void update(HapticDisplay *display);
};

#endif // MODIFIEDMORRISSURFACE_H
