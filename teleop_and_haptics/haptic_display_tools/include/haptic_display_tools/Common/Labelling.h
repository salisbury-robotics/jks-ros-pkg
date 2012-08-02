#ifndef LABELLING_H
#define LABELLING_H

#include <vector>
#include <cml/mathlib/typedef.h>

struct Labelling
{
    bool    dirty;          // indicates label field changed -> need to save
    bool    combine;        // whether or not to combine this volume with master
    bool    processed;      // post-processed and ready for volume rendering

    bool    threshold;      // whether or not to threshold the volume
    int     thresholdValue; // level at which to threshold

    // lookup table of RGBA colours, first entry corresponds to intensity 1
    std::vector<cml::vector4f> colourLUT;

    Labelling() :
            dirty(false), combine(false), processed(false),
            threshold(false), thresholdValue(0)
    { }

    void initializeLUT(int n) {
        const int k_maxEntries = 24;
        n = std::min(n, k_maxEntries);
        colourLUT = std::vector<cml::vector4f>(n, cml::vector4f(0.f, 0.f, 0.f, 0.f));
    }

    int entries() const {
        return colourLUT.size();
    }
};

#endif // LABELLING_H
