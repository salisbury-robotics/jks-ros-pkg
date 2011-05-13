#ifndef HAPTICISOSURFACE_H
#define HAPTICISOSURFACE_H

#include "Common/VolumeSampler.h"
#include "Common/VolumeCutter.h"
#include "HapticScene.h"

class HapticIsosurface : public HapticNode
{
protected:
    static const double k_stepTangent;
    static const double k_stepNormal;
    static const int    k_maxStates = 4;

    VolumeSampler  *m_sampler;
    double          m_isoValue;

    // volume cutter instance to handle removing material
    VolumeCutter   *m_maskCutter;
    double          m_toolRadius;

    // a structure to track the state of a proxy for a given haptic device
    struct State {
        bool            inside;
        bool            ready;
        cml::vector3d   proxy;
        State() : inside(false), ready(false), proxy(0.0, 0.0, 0.0) { }
    } m_states[k_maxStates];

    // uses the isovalue to 0-center the field, values < 0 are inside object
    double surface(const cml::vector3d &p)
        { return m_isoValue - m_sampler->intensityAt(p); }

    // the field gradient is the negative of the volume gradient
    cml::vector3d gradient(const cml::vector3d &p)
        { return -m_sampler->gradientAt(p); }

    // finds the surface within the field (Salisbury & Tarr 1998)
    cml::vector3d findSurface(const cml::vector3d &seed);

    // alternate method to find the surface, given seed and inward direction
    // (magnitude also used as the distance to look)
    cml::vector3d findSurface(const cml::vector3d &seed,
                              const cml::vector3d &direction);

public:
    HapticIsosurface(Volume *volume, Volume *mask = 0, double isoValue = 0.5f);
    virtual ~HapticIsosurface();

    virtual void setGradientDelta(double delta)
        { m_sampler->setGradientDelta(delta); }

    // warning: best to pause the haptic servo when changing this
    virtual void setIsosurfaceValue(double value);

    // sets the tool radius (for cutting)
    virtual void setToolRadius(double r)
        { m_toolRadius = r; }

    // this update function direct sets a proxy position on the display
    virtual void update(HapticDisplay *display);

    // resets the proxy tracking to an uninitialized state
    virtual void reset();

    // use these methods to query for changes to a region of the mask
    virtual bool maskAltered()          { return m_maskCutter->regionAltered(); }
    virtual void maskFetchAndResetRegion(cml::vector3i &lower, cml::vector3i &upper)
        { m_maskCutter->regionFetchAndReset(lower, upper); }
};

#endif // HAPTICISOSURFACE_H
