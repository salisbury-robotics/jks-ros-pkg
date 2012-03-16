//---------------------------------------------------------------------------
#ifdef _MSVC
#pragma warning (disable : 4786)
#endif
//---------------------------------------------------------------------------
#ifndef CGENERICSAMPLEDGEOMETRY_H
#define CGENERICSAMPLEDGEOMETRY_H
//---------------------------------------------------------------------------
#include "world/CGenericObject.h"
#include "eigen/Eigen"
#include <vector>
#include <list>
//---------------------------------------------------------------------------
class cWorld;
//---------------------------------------------------------------------------

namespace srl {

class GenericSampledGeometry : public cGenericObject
{
public:
  GenericSampledGeometry()    {};
  virtual ~GenericSampledGeometry()   {};

  virtual float             intensityAt(const Eigen::Vector3f) = 0;
  virtual Eigen::Vector3f   gradientAt( const Eigen::Vector3f) = 0;
};

}

#endif // CGENERICSAMPLEDGEOMETRY_H
