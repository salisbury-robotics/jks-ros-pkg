#ifndef _SAMPLER_H_
#define _SAMPLER_H_

#include <cml/mathlib/typedef.h>
#include <limits>

// The Sampler performs trilinear interpolation to sample a volume in
// 0-1 cube space with correct physical proportions (ie. the maximum physical
// dimension of the volume is scaled to fit in the range 0,1).  Index
// coordinates are double-precision values, and the returned samples are
// single-precision floats.  Templatization for other variations may be
// developed later if needed.

class Sampler
{


public:

  virtual ~Sampler()  {};


  virtual float           intensityAt(const cml::vector3d &p) = 0;
  virtual cml::vector3f   gradientAt(const cml::vector3d &p) = 0;
};

#endif // _SAMPLER_H_
