
///\author Adam Leeper
///\brief Node for simple haptic interaction

#include "chai3d.h"

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

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
#include <Common/PointSampler.h>

#include <dynamic_reconfigure/server.h>
#include <haptic_ghosted_gripper/HapticsConfig.h>

//typedef pcl::PointXYZRGB PointT;


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
  ros::Publisher pub_pose_;

  //! mutex for point cloud publishing
  boost::mutex mutex_;

  // A timer callback
  ros::Timer update_timer_;

  // ***** Dynamic reconfigure stuff *****
  typedef haptic_ghosted_gripper::HapticsConfig Config;
  Config config_;
  dynamic_reconfigure::Server<Config>                dyn_srv;
  dynamic_reconfigure::Server<Config>::CallbackType  dyn_cb;

  //! Tells if haptic stuff is in use.
  bool active_;

  //! Stuff for tf
  tf::TransformListener tfl_;
  tf::TransformBroadcaster tfb_;
  std::string m_display_frame;

  // label to show estimate of haptic update rate
  double fps_estimate;
  double hps_estimate;

  //! A haptic scene containing the isosurface object and a display.
  HapticScene            *m_scene;

  //! An index for the haptic scene to use... we only use one...
  int                     m_sceneIndex;

  //! The sampler for the object to render.
  Sampler                *m_sampler;

  //! The mesh for the proxy (only for graphical rendering... probably not needed.
  MeshGLM                *m_mesh;

  //! Direct hardware handler for the haptic device
  cHapticDeviceHandler   *m_handler;

  //! Info for the haptic device
  cHapticDeviceInfo       m_device_info;

  static const int k_clickButton = 0;
  bool m_clicking;
  bool m_clickExternal;

public:
  HapticGhostedGripper();
  ~HapticGhostedGripper();


  // TODO These are public because it is easier to directly access them...
  //! The haptic display (device) interface class
  HapticDisplay          *m_display;

  //! The isosurface class
  HapticIsosurface       *m_isosurface;


  void initializeHaptics();
  void displayCallback();
  void printDeviceSpecs(const cHapticDeviceInfo &info);
  void loadPointShell(const std::string &location, float scale);
  void publishPointCloud();
  void dynamicCallback(Config &new_config, uint32_t id);

  void startHaptics();
  void stopHaptics();
  void loadPointCloud( pcl::PointCloud<PointT>::Ptr &cloud );
  bool checkForButtonClick();





};
