
///\author Adam Leeper
///\brief Node for simple haptic interaction

#include "chai3d.h"

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

#include <boost/bind.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <object_manipulator/tools/shape_tools.h>

#include <Haptics/HapticDisplay.h>
#include <Haptics/HapticIsosurface.h>
#include <Graphics/Mesh.h>

#include <iostream>
#include <fstream>
#include <queue>

#include <Common/Sampler.h>


//#define PROF_ENABLED
//#include <profiling/profiling.h>
//PROF_DECLARE(TOTAL_TIMER)
//PROF_DECLARE(FUNC_1)


class HapticGhostedGripper{

  //! Node handles
  ros::NodeHandle nh_, pnh_;

  //! Publishers
  ros::Publisher pub_marker_, pub_marker_array_;
  ros::Publisher pub_status_;

  // A timer callback
  ros::Timer update_timer_;

  //! Stuff for tf
  tf::TransformListener tfl_;
  tf::TransformBroadcaster tfb_;
  std::string m_display_frame;

  // label to show estimate of haptic update rate
  double fps_estimate;
  double hps_estimate;

  HapticScene            *m_scene;
  HapticIsosurface       *m_isosurface;
  int                     m_sceneIndex;
  HapticDisplay          *m_display;
  Sampler                *m_sampler;
  MeshGLM                *m_mesh;



  cHapticDeviceHandler   *m_handler;
  cHapticDeviceInfo       m_device_info;

public:
  HapticGhostedGripper();
  ~HapticGhostedGripper();


  void initializeHaptics();
  void displayCallback();
  void printDeviceSpecs(const cHapticDeviceInfo &info);
  void loadPointShell(const std::string &location);




};
