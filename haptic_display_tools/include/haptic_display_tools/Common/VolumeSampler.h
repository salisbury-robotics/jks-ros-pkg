#ifndef _VOLUMESAMPLER_H_
#define _VOLUMESAMPLER_H_

#include "Sampler.h"
#include "Volume.h"
#include <limits>

// The VolumeSampler performs trilinear interpolation to sample a volume in
// 0-1 cube space with correct physical proportions (ie. the maximum physical
// dimension of the volume is scaled to fit in the range 0,1).  Index
// coordinates are double-precision values, and the returned samples are
// single-precision floats.  Templatization for other variations may be
// developed later if needed.

class VolumeSampler : public Sampler
{
    Volume         *m_volume;
    Volume         *m_mask;

    // whether or not to multiply by the mask value for returned intensity
    bool            m_useMask;

    // volume extents in each direction (1.0 max)
    cml::vector3d   m_extents;

    // bounds on the voxel indices of the volume
    cml::vector3i   m_lower, m_upper;

    // a scale and bias of the floating point range returned
    float           m_scale, m_bias;

    // delta step size for central difference estimation of intensity gradient
    double          m_gradientDelta;

    // index the volume to retrieve a voxel intensity (no error-checking)
    template <class T>
    float voxel(int x, int y, int z) {
        const cml::vector3i &d = m_volume->dimensions;
        const T *p = reinterpret_cast<const T *>(m_volume->data);
        p += (z * d[1] + y) * d[0] + x;
        return float(*p) / std::numeric_limits<T>::max();
    }

    // index the mask to retrieve a mask intensity
    float mask(int x, int y, int z) {
        const cml::vector3i &d = m_mask->dimensions;
        const unsigned char *p = reinterpret_cast<const unsigned char *>(m_mask->data);
        p += (z * d[1] + y) * d[0] + x;
        return float(*p) / std::numeric_limits<unsigned char>::max();
    }

    // pointer to the instantiation of the indexing function for the current type
    float (VolumeSampler::*m_voxel)(int x, int y, int z);

public:
    VolumeSampler(Volume *v = 0, Volume *m = 0, double gdelta = 1.0 / 128.0);

    void setVolume(Volume *v, Volume *m = 0);
    void setGradientDelta(double d) { m_gradientDelta = d; }
    void setUseMask(bool b)         { m_useMask = b; }

    float           intensityAt(const cml::vector3d &p);
    cml::vector3f   gradientAt(const cml::vector3d &p);
};

#endif // VOLUMESAMPLER_H
