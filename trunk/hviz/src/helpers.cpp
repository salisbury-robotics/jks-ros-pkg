#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "helpers.h"


namespace chai_tools
{

tf::Vector3 cVectorToTF(const cVector3d &v)
{
  tf::Vector3 vtf;
  vtf.setX(v.x);
  vtf.setY(v.y);
  vtf.setZ(v.z);
  return vtf;
}

btMatrix3x3 cMatrixToTF(const cMatrix3d &rot)
{
  return btMatrix3x3( rot.getCol0().x, rot.getCol1().x, rot.getCol2().x,
                      rot.getCol0().y, rot.getCol1().y, rot.getCol2().y,
                      rot.getCol0().z, rot.getCol1().z, rot.getCol2().z);
}

tf::Quaternion cMatrixToTFQuaternion(const cMatrix3d &m)
{
  tf::Quaternion q;
  cMatrixToTF(m).getRotation(q);
  return q;
}

} // namespace chai_tools

