#include "VolumeCutter.h"
#include <QMutex>
#include <cml/cml.h>

using namespace cml;

// --------------------------------------------------------------------------

VolumeCutter::VolumeCutter(Volume *volume)
{
    m_mutex = new QMutex();
    setVolume(volume);
}

VolumeCutter::~VolumeCutter()
{
    if (m_mutex) delete m_mutex;
}

// --------------------------------------------------------------------------

void VolumeCutter::setVolume(Volume *v)
{
    m_volume = v;
    if (v) {
        // compute extents of the volume inside the 0-1 cube
        vector3f p = v->physical();
        int ix = index_of_max(p[0], p[1], p[2]);
        m_extents = p / p[ix];

        // set in the index bounds of the volume
        m_lower = vector3i(0, 0, 0);
        m_upper = v->dimensions - vector3i(1, 1, 1);

        // compute helpers to convert form x,y,z to offset
        m_bytesPerLine = v->dimensions[0];
        m_bytesPerSlice = v->dimensions[1] * m_bytesPerLine;

        // compute scale factors between image coordinates and world
        m_scaleToImage.set(m_volume->dimensions[0] / m_extents[0],
                           m_volume->dimensions[1] / m_extents[1],
                           m_volume->dimensions[2] / m_extents[2]);
        m_scaleToWorld.set(m_extents[0] / m_volume->dimensions[0],
                           m_extents[1] / m_volume->dimensions[1],
                           m_extents[2] / m_volume->dimensions[2]);

        // set sigma to center-to-corner distance of a voxel
//        m_sigma = m_scaleToWorld.length();
        m_sigma = 0.5 * (m_scaleToWorld[0] + m_scaleToWorld[1] + m_scaleToWorld[2]);

        // initialize region tracking variables
        m_regionLower = m_volume->dimensions;
        m_regionUpper = zero_3D();
    }
}

// --------------------------------------------------------------------------

void VolumeCutter::cutFilteredSphere(const cml::vector3d &center, double radius)
{
    // figure out which voxels are within the radius
    vector3i r = radius * m_scaleToImage;
    vector3i c = toImage(center);
    vector3i a = c - r - vector3i(1,1,1);
    vector3i b = c + r + vector3i(1,1,1);

    // bound the voxels to the image dimensions
    a.minimize(m_upper);
    b.maximize(m_lower);

    // variables to track the actual changed region
    vector3i ra = m_upper, rb = m_lower;

    // iterate through the sub-volume and update the mask
    for (int z = a[2]; z <= b[2]; ++z)
        for (int y = a[1]; y <= b[1]; ++y)
            for (int x = a[0]; x <= b[0]; ++x)
            {
                vector3i p(x, y, z);
                double d = (toWorld(p) - center).length();
                unsigned char v = filterValue(radius, d);
                if (minimize(p, v)) {
                    ra.minimize(p);
                    rb.maximize(p);
                    m_volume->modified = true;
                }
            }

    // update the region tracking variables
    if (ra <= rb) {
        m_mutex->lock();
        m_regionLower.minimize(ra);
        m_regionUpper.maximize(rb);
        m_mutex->unlock();
    }
}

// --------------------------------------------------------------------------

void VolumeCutter::regionFetchAndReset(cml::vector3i &lower, cml::vector3i &upper)
{
    m_mutex->lock();
    lower = m_regionLower;
    upper = m_regionUpper;
    m_regionLower = m_volume->dimensions;
    m_regionUpper = zero_3D();
    m_mutex->unlock();
}

// --------------------------------------------------------------------------
