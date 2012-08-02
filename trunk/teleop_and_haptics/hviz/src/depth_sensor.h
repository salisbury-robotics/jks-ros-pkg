#ifndef DEPTH_SENSOR_H
#define DEPTH_SENSOR_H

#include "chai3d.h"
#include "cplane.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//#include <pcl/features/normal_3d.h>
//
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/passthrough.h>
//
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>

#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>

typedef pcl::PointXYZ PointT;

class DepthSensor : public cMesh
{

  stereo_msgs::DisparityImage m_image;
  sensor_msgs::CameraInfo m_camera_info;

public:
    DepthSensor(cWorld *a_world);

    void getNeighborPoints(float x, float y, float z, std::vector<PointT> &neighbors);

    cPlane *tPlane;

};

#endif // DEPTH_SENSOR_H
