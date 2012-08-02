#include "TransferFunction.h"
#include <cml/cml.h>

using namespace std;
using namespace cml;

const vector4f TransferFunction::k_invalid = vector4f(-1.f, -1.f, -1.f, 0.f);

void TransferFunction::computeRGBA8(unsigned char *output, int size)
{
    // maker sure there is a marker at position 0 and 1
    if (markers.find(0.f) == markers.end())
        markers[0.f] = ColourPair(zero_4D(), zero_4D());
    if (markers.find(1.f) == markers.end())
        markers[1.f] = ColourPair(zero_4D(), zero_4D());

    float a, b, step = 1.f / size;
    vector4f ca, cb;
    map<float, ColourPair>::iterator it = markers.begin();    
    for (float position = 0.5f * step; position < 1.f; position += step)
    {
        // advance to the next marker if needed
        while (it->first < position) {
            a = it->first;
            ca = it->second.second;
            ++it;
            b = it->first;
            cb = it->second.first;
        }

        // compute an interpolation for the current position
        float t = (a != b) ? (position - a) / (b - a) : 0.5f;
        vector4f c = lerp(ca, cb, t);

        // convert to single bytes and store
        for (int i = 0; i < 4; ++i)
            *output++ = (unsigned char)(clamp(c[i], 0.f, 1.f) * 255.f + .5f);
    }
}

