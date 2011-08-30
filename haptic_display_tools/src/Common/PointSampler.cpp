#include "PointSampler.h"
#include <algorithm>

using namespace cml;

PointSampler::PointSampler()
{

}

float PointSampler::intensityAt(const cml::vector3d &p)
{
  float radius = 1.0;
  return radius - p.length();
}

vector3f PointSampler::gradientAt(const cml::vector3d &p)
{

  return -2*p;

  /* Numerical method, if we care
    vector3d dx(m_gradientDelta, 0.0, 0.0);
    vector3d dy(0.0, m_gradientDelta, 0.0);
    vector3d dz(0.0, 0.0, m_gradientDelta);

    vector3f g;
    g[0] = intensityAt(p + dx) - intensityAt(p - dx);
    g[1] = intensityAt(p + dy) - intensityAt(p - dy);
    g[2] = intensityAt(p + dz) - intensityAt(p - dz);

    return 0.5 * g / m_gradientDelta;
    */

}
