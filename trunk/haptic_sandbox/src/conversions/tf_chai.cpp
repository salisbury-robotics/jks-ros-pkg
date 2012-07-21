
#include <conversions/tf_chai.h>


namespace tf
{

tf::Vector3 vectorChaiToTf(const cVector3d &v)
{
  tf::Vector3 vtf;
  vtf.setX(v.x());
  vtf.setY(v.y());
  vtf.setZ(v.z());
  return vtf;
}

tf::Matrix3x3 matrixChaiToTf(const cMatrix3d &rot)
{
    return tf::Matrix3x3( rot.getCol0().x(), rot.getCol1().x(), rot.getCol2().x(),
                          rot.getCol0().y(), rot.getCol1().y(), rot.getCol2().y(),
                          rot.getCol0().z(), rot.getCol1().z(), rot.getCol2().z());
}

tf::Quaternion quaternionChaiMatrixToTf(const cMatrix3d &m)
{
  tf::Quaternion q;
  matrixChaiToTf(m).getRotation(q);
  return q;
}

} // namespace chai_tools

