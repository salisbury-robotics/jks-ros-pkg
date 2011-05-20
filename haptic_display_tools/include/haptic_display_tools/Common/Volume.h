#ifndef VOLUME_H
#define VOLUME_H

#include <string>
#include <vector>
#include <limits>
#include <algorithm>
#include <cml/mathlib/typedef.h>

struct Histogram
{
    int             minRange, maxRange; // the limits of the data type
    int             minValue, maxValue; // the actual min/max values present
    int             maxCount, maxNonZero;
    std::vector<int> counts;            // counts for each of [minR, maxR]

    int valueRange() const { return maxValue - minValue; }
};

struct Volume
{
    enum PixelFormat { pfUInt8, pfInt16, pfUInt16, pfSentinel };

    std::string     name;
    std::string     path;
    cml::vector3i   dimensions;
    cml::vector3f   spacing;
    PixelFormat     format;
    void           *data;
    bool            modified;
    Histogram       histogram;

    // transformation from the 0-1 cube to the volume's physical coordinates
    cml::matrix44f_c transform;

    // a constructor that sets up some reasonable defaults..
    Volume() :
            dimensions(cml::vector3i(0, 0, 0)),
            spacing(cml::vector3f(1.f, 1.f, 1.f)),
            format(pfSentinel), data(0), modified(false),
            transform(cml::matrix44f_c(1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1))
    { }

    // physical dimensions - derived quanity
    cml::vector3f physical() const {
        return cml::vector3f(dimensions[0] * spacing[0],
                             dimensions[1] * spacing[1],
                             dimensions[2] * spacing[2]);
    }

    // bytes per voxel - derived from format
    int bytesPerVoxel() const {
        return (format == pfUInt8) ? 1 : 2;
    }

    void computeHistogram()
    {
        // dispatch the templated member based on format type
        switch(format) {
        case pfUInt8:
            compute<unsigned char>(); break;
        case pfInt16:
            compute<signed short>(); break;
        case pfUInt16:
            compute<unsigned short>(); break;
        default:
            break;
        }
    }

protected:
    template <class T> void compute()
    {
        Histogram &h = histogram;

        h.minRange = std::numeric_limits<T>::min();
        h.maxRange = std::numeric_limits<T>::max();

        // set initial min/max values
        h.minValue = h.maxRange;
        h.maxValue = h.minRange;

        histogram.counts = std::vector<int>(h.maxRange-h.minRange+1, 0);
        const T *d = reinterpret_cast<const T *>(data);

        int limit = dimensions[0] * dimensions[1] * dimensions[2];
        for (int i = 0; i < limit; ++i, ++d) {
            ++histogram.counts[*d - h.minRange];
            h.minValue = std::min<int>(h.minValue, *d);
            h.maxValue = std::max<int>(h.maxValue, *d);
        }

        // compute the biggest bucket stats
        int t = h.counts[-h.minRange];
        h.counts[-h.minRange] = 0;
        h.maxNonZero = *std::max_element(h.counts.begin(), h.counts.end());
        h.maxCount = std::max(h.maxNonZero, t);
        h.counts[-h.minRange] = t;
    }
};

#endif // VOLUME_H
