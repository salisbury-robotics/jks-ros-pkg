#ifndef TRANSFERFUNCTION_H
#define TRANSFERFUNCTION_H

#include <map>
#include <cml/mathlib/typedef.h>

struct TransferFunction
{
    static const cml::vector4f k_invalid;

    typedef std::pair< cml::vector4f, cml::vector4f > ColourPair;
    std::map<float, ColourPair> markers;

    // inserts
    void insert(float position, cml::vector4f colourL,
                                cml::vector4f colourR = k_invalid)
    {
        if (colourR == k_invalid) colourR = colourL;
        markers[position] = ColourPair(colourL, colourR);
    }

    void computeRGBA8(unsigned char *output, int size);
};

#endif // TRANSFERFUNCTION_H
