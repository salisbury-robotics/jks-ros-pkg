#include "VolumeSampler.h"
#include <cml/mathlib/interpolation.h>
#include <algorithm>

using namespace cml;

VolumeSampler::VolumeSampler(Volume *v, Volume *m, double gdelta)
{
    m_useMask = (m != 0);
    m_gradientDelta = gdelta;
    setVolume(v, m);
}

void VolumeSampler::setVolume(Volume *v, Volume *m)
{
    m_volume = v;
    m_mask = m;

    if (v) {
        // compute extents of the volume inside the 0-1 cube
        vector3f p = v->physical();
        int ix = index_of_max(p[0], p[1], p[2]);
        m_extents = p / p[ix];

        // set in the index bounds of the volume
        m_lower = vector3i(0, 0, 0);
        m_upper = v->dimensions - vector3i(1, 1, 1);

        // select the correct instantiation of the indexing function
        switch (v->format) {
        case Volume::pfUInt8:   m_voxel = &VolumeSampler::voxel<unsigned char>; break;
        case Volume::pfInt16:   m_voxel = &VolumeSampler::voxel<short>;         break;
        case Volume::pfUInt16:  m_voxel = &VolumeSampler::voxel<unsigned short>;break;
        }

        // determine the scale and bias based on the volume pixel format
        switch (v->format) {
        case Volume::pfInt16:
            m_scale = 0.5f;
            m_bias = 0.5f;
        default:
            m_scale = 1.f;
            m_bias = 0.f;
            break;
        }

        // TODO: special case to detect likely CT scans in Hounsfield Units
        //       (mirrors the code in VolumeRenderer.cpp)
        if (v->format == Volume::pfInt16 &&
            v->histogram.minValue > -4096 &&
            v->histogram.maxValue < 4096)
        {
            m_scale = 8.f;
            m_bias = 0.25f;
        }
    }

    if (m == 0) m_useMask = false;
}

float VolumeSampler::intensityAt(const cml::vector3d &p)
{
    // return zero in absense of volume
    if (m_volume == 0) return 0.f;

    // quick bounds check
    for (int i = 0; i < 3; ++i)
        if (!in_range(p[i], 0.0, m_extents[i])) return -1.f;

    // compute image coordinates from p
    vector3d coord(m_volume->dimensions[0] * p[0] / m_extents[0],
                   m_volume->dimensions[1] * p[1] / m_extents[1],
                   m_volume->dimensions[2] * p[2] / m_extents[2]);
    coord += vector3d(.5, .5, .5);

    vector3i b = coord;                 // integer upper corner
    vector3i a = b - vector3i(1, 1, 1); // integer lower corner
    vector3d c = coord - b;             // fractional remainder

    // ensure that our samples will be within image bounds
    b.minimize(m_upper);
    a.maximize(m_lower);

    // sample the volume at 8 corners
    float f[8];
    for (int i = 0; i < 8; ++i)
    {
        // compute sample x,y,z indices
        int x = (i&1) ? b[0] : a[0];
        int y = (i&2) ? b[1] : a[1];
        int z = (i&4) ? b[2] : a[2];

        // retrieve sample from volume
        f[i] = (this->*m_voxel)(x, y, z);

        // apply scale and bias
        f[i] = f[i] * m_scale + m_bias;

        // apply mask if requested (applied after scale/bias)
        if (m_useMask) f[i] *= mask(x, y, z);
    }

    // perform tri-linear interpolation on the samples
    return trilerp(f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], c[0], c[1], c[2]);
}

vector3f VolumeSampler::gradientAt(const cml::vector3d &p)
{
    vector3d dx(m_gradientDelta, 0.0, 0.0);
    vector3d dy(0.0, m_gradientDelta, 0.0);
    vector3d dz(0.0, 0.0, m_gradientDelta);

    vector3f g;
    g[0] = intensityAt(p + dx) - intensityAt(p - dx);
    g[1] = intensityAt(p + dy) - intensityAt(p - dy);
    g[2] = intensityAt(p + dz) - intensityAt(p - dz);

    return 0.5 * g / m_gradientDelta;
}
