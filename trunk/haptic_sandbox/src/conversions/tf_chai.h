#ifndef _TF_CHAI_H_
#define _TF_CHAI_H_


#include <tf/tf.h>
#include <chai3d.h>

namespace tf
{

  tf::Vector3 vectorChaiToTf(const cVector3d &v);
  cVector3d   vectorTfToChai(const tf::Vector3 &v);

  tf::Matrix3x3 matrixChaiToTf(const cMatrix3d &rot);
  cMatrix3d matrixTfToChai(const cMatrix3d &rot);

  tf::Quaternion quaternionChaiMatrixToTf(const cMatrix3d &m);
  cMatrix3d quaternionTfToChaiMatrix(const tf::Quaternion &q);

} // namespace tf


#endif
