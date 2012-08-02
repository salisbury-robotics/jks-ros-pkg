#ifndef VOLUMECUTTER_H
#define VOLUMECUTTER_H

#include "Volume.h"

// forward-declare a mutex for locking the changed region state
class QMutex;

class VolumeCutter
{
    // the volume must be an 8-bit unsigned mask volume
    Volume         *m_volume;

    // volume extents in each direction (1.0 max)
    cml::vector3d   m_extents;

    // bounds on the voxel indices of the volume
    cml::vector3i   m_lower, m_upper;

    // for going from x,y,z to image offset, since we will do that so much
    int             m_bytesPerLine, m_bytesPerSlice;

    // controls the amount of smoothing on edge of cut (depends on voxel size)
    double          m_sigma;

    // variables for tracking the changed region so that the visualization can
    // be updated periodically
    cml::vector3i   m_regionLower, m_regionUpper;
    QMutex         *m_mutex;

    // conversion scales between image and 0-1 cube space
    cml::vector3d   m_scaleToImage;
    cml::vector3d   m_scaleToWorld;

    // coordinate conversion helper functions
    cml::vector3i   toImage(const cml::vector3d &p) {
                        return cml::vector3i(p[0] * m_scaleToImage[0],
                                             p[1] * m_scaleToImage[1],
                                             p[2] * m_scaleToImage[2]);
                    }
    cml::vector3d   toWorld(const cml::vector3i &p) {
                        return cml::vector3d((p[0]+0.5) * m_scaleToWorld[0],
                                             (p[1]+0.5) * m_scaleToWorld[1],
                                             (p[2]+0.5) * m_scaleToWorld[2]);
                    }

    // sets the value at given image coordinate if new value is smaller
    bool minimize(const cml::vector3i p, unsigned char value)
    {
        int offset = p[0] + p[1] * m_bytesPerLine + p[2] * m_bytesPerSlice;
        unsigned char *voxel = reinterpret_cast<unsigned char *>(m_volume->data) + offset;
        if (value < *voxel) { *voxel = value; return true; }
        return false;
    }

    // computes a filtered mask value based on radial distance and sigma
    unsigned char filterValue(double r, double d)
    {
        double s = std::min(r, m_sigma);
        if      (d <= r - s) return 0;
        else if (d >= r)     return 0xff;
        else {
            // symmetric cubic ease function (better than Bartlett?)
            double t = 1.0 - (r - d) / s;
            return static_cast<unsigned char>(t*t*(3-2*t) * 255.0 + 0.5);
        }
    }

public:
    VolumeCutter(Volume *volume);
    ~VolumeCutter();

    void setVolume(Volume *v);

    void cutFilteredSphere(const cml::vector3d &center, double radius);

    // for querying the altered region
    bool regionAltered()    { return m_regionLower <= m_regionUpper; }
    void regionFetchAndReset(cml::vector3i &lower, cml::vector3i &upper);
};

#endif // VOLUMECUTTER_H
