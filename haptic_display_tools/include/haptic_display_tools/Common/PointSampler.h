#ifndef _POINTSAMPLER_H_
#define _POINTSAMPLER_H_

#include "Sampler.h"

//#include pcl stuff

// The VolumeSampler performs trilinear interpolation to sample a volume in
// 0-1 cube space with correct physical proportions (ie. the maximum physical
// dimension of the volume is scaled to fit in the range 0,1).  Index
// coordinates are double-precision values, and the returned samples are
// single-precision floats.  Templatization for other variations may be
// developed later if needed.

class PointSampler : public Sampler
{

  float metaball_radius;
  float threshold;
  float other_stuff;

public:

  PointSampler();

  virtual float           intensityAt(const cml::vector3d &p);
  virtual cml::vector3f   gradientAt(const cml::vector3d &p);
};

#endif // VOLUMESAMPLER_H
