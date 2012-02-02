#include "depth_sensor.h"


#include <rviz_interaction_tools/image_tools.h>

DepthSensor::DepthSensor(cWorld *a_world)
        : cMesh(a_world)
//        , m_projectedSphere(0.05), tree(0),
//          pclTree (new pcl::KdTreeFLANN<PointT> ()), m_shape(0), scaleZ(1.0), m_tool(0)
{
    // because we are haptically rendering this object as an implicit surface
    // rather than a set of polygons, we will not need a collision detector
    setCollisionDetector(0);

//    m_inContact = false;
//    m_showTangent = true;
//    m_useFriction = true;
//    m_isReady = false;
//    m_normalsFlip = false;

    //----------------------------
    // Tangent plane
    tPlane = new cPlane(a_world);
    this->addChild(tPlane);

    // --------------------------------------------

//    // create a sphere that tracks the position of the proxy
//    m_projectedSphere.m_material.m_ambient.set(0.3, 0.3, 0.0);
//    m_projectedSphere.m_material.m_diffuse.set(0.8, 0.8, 0.0);
//    m_projectedSphere.m_material.m_specular.set(1.0, 1.0, 1.0);
//    addChild(&m_projectedSphere);
}







void DepthSensor::getNeighborPoints(float x, float y, float z, std::vector<PointT> &neighbors)
{

}
