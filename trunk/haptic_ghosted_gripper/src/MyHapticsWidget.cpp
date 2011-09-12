

#include "MyHapticsWidget.h"
#include "cml_conversions.h"

#include "MyHapticsThread.h"

#include <Haptics/PointShellIsosurface.h>
#include <Common/PointSampler.h>



  //! The constructor
HapticGhostedGripper::HapticGhostedGripper():
    nh_("/"), pnh_("~"),
    m_display_frame("/base_link")
{
  printf("Why isn't this constructing?\n");

  ROS_INFO("Constructing HapticGhostedGripper!");
  // Marker topics
  pub_marker_ = pnh_.advertise<visualization_msgs::Marker>("/markers", 100);
  pub_marker_array_ = pnh_.advertise<visualization_msgs::MarkerArray>("/markers_array", 1);

  // String status topic
  pub_status_ = pnh_.advertise<std_msgs::String>("status", 10);

  // Create timer callback for auto-running of algorithm
  update_timer_ = nh_.createTimer(ros::Duration(0.03333), boost::bind(&HapticGhostedGripper::displayCallback, this));

  // Set up the chai world and device
  initializeHaptics();

//    m_sceneIndex = hthread->addScene(m_scene);
//    hthread->selectScene(m_sceneIndex);

  ROS_INFO("Finished constructor!");
}

HapticGhostedGripper::~HapticGhostedGripper()
{
  MyHapticsThread *hthread = MyHapticsThread::instance();
  hthread->quit();
}

void HapticGhostedGripper::printDeviceSpecs(const cHapticDeviceInfo &info)
{
  ROS_INFO("Model: %s", info.m_modelName.c_str());
  ROS_INFO("Max Force: %.1f N", info.m_maxForce);
  ROS_INFO("Max Torque: %.1f Nm", info.m_maxTorque);
  ROS_INFO("Max Stiffness: %.f N/m", info.m_maxForceStiffness);
  ROS_INFO("Workspace: %.3f m", info.m_workspaceRadius);
}

void HapticGhostedGripper::loadPointShell(const std::string &location)
{
    PointShellIsosurface *surface = dynamic_cast<PointShellIsosurface *>(m_isosurface);
    if (surface == 0) return;

//    string directory = location.toStdString();
    MeshGLM mesh("pointshell", "/opt/ros/electric/stacks/pr2_common/pr2_description/meshes/gripper_v0/gripper_palm.dae");


    // use the mesh vertices as a point shell on the haptic isosurface
    surface->setPointShell(mesh.vertexPointer(), mesh.numVertices());
}


void HapticGhostedGripper::initializeHaptics()
{
    ROS_INFO("Creating connection to device...");

    // create a haptic device handler that tells us what devices are connected
    // to the host computer
    m_handler = new cHapticDeviceHandler();

    // initialize all available haptic devices here
    ROS_INFO("Found %d attached devices!", m_handler->getNumDevices());
//    for (int i = 0; i < m_handler->getNumDevices(); ++i) {
//        cGenericHapticDevice *device;
//        if (m_handler->getDevice(device, i) == 0) {
//            device->open();
////            device->initialize(true);
//        }
//    }

    // get access to the first available haptic device
    cGenericHapticDevice* hapticDevice;
    m_handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    if (hapticDevice)
    {
        m_device_info = hapticDevice->getSpecifications();
        printDeviceSpecs(m_device_info);
    }

//    m_display = new HapticDisplay(hapticDevice, 0);
//    m_display->setToolRadius(0.08);
//    m_display->setWorkspaceRadius(m_display->deviceWorkspace());
//    m_display->setStiffness(m_device_info.m_maxForceStiffness * 0.5);

//    cml::matrix33d permutation;
//    permutation.identity();
//    m_display->setPermutation(permutation);

    // create a haptics scene with all the haptic displays in it
    m_scene = new HapticScene();

    if (hapticDevice)
    {
        HapticDisplay *display = m_scene->addHapticDisplay(hapticDevice);
        m_display = display; //.append(display);

//        // center the device at (.5, .5, .5) where the volume will be
        cml::matrix44f_c center, orient = cml::identity_4x4();
        cml::matrix_translation(center, 0.f, -.5f, 0.f);
        cml::matrix_set_basis_vectors(orient, cml::z_axis_3D(), cml::x_axis_3D(), cml::y_axis_3D());
        display->setTransform(orient * center);

        // TODO: increase stiffness when speed problem is solved
        double k = display->stiffness();
        display->setStiffness(0.5 * k);
    }


//    // save obj models to disk so we can load them back
//    QString location = expandModelResources();
//    string directory = location.toStdString();
//    m_drill = new MeshGLM("drill", directory + "/drill.obj",
//                          MeshGLM::k_useTexture | MeshGLM::k_useMaterial);
//    m_burr = new MeshGLM("burr", directory + "/burr1.obj",
//                         MeshGLM::k_useMaterial);


    m_sampler = new PointSampler();
    PointSampler *ps = dynamic_cast<PointSampler*>(m_sampler);
    ps->createSphericalShell(0.5, 0.05, -1, 1, -M_PI, M_PI, 0.0);
    //ps->createPlane();
    ps->applyLastCloud();

    m_mesh = new MeshGLM("drill", "/opt/ros/electric/stacks/pr2_common/pr2_description/meshes/gripper_v0/gripper_palm.dae", MeshGLM::k_useTexture | MeshGLM::k_useMaterial);


    // add a haptic isosurface to the scene
    //m_isosurface = new HapticIsosurface(m_sampler, 0.5);
    m_isosurface = new PointShellIsosurface(m_sampler, 0.5);
    loadPointShell("");
    m_scene->addNode(m_isosurface);

    // clean up the temporary model files from the disk
    //removeModelResources(location);

    // add the scene to the haptics thread and select it for rendering
    MyHapticsThread *hthread = MyHapticsThread::instance();
    m_sceneIndex = hthread->addScene(m_scene);
    hthread->selectScene(m_sceneIndex);
    if(!hthread->resume())
    {
      ROS_ERROR("Failed to resume haptics thread...something is very wrong!");
    }

}


//! The display callback sends graphical output to rviz.
void HapticGhostedGripper::displayCallback()
{
  if(!m_display) return;

  ros::Time time_now = ros::Time::now(); // use single time for all output

  static bool firstTime = true;
  static int counter = 0;
  static cPrecisionClock pclock;

  if(firstTime) // start a clock to estimate the rate
  {
      pclock.setTimeoutPeriodSeconds(1.0);
      pclock.start(true);
      firstTime = false;
  }

  // estimate the refresh rate and publish
  ++counter;
  if (pclock.timeoutOccurred()) {
      pclock.stop();
      fps_estimate = counter;
      counter = 0;
      pclock.start(true);
      std_msgs::String msg;
      char status_string[256];
      sprintf(status_string, "Haptic rate: %.3f Graphics Rate: %.3f",
              hps_estimate, fps_estimate);
      //ROS_INFO_STREAM(status_string);
      msg.data = status_string;
      pub_status_.publish(msg);
  }

  // Transmit the visualizations of the tool/proxy
  float proxy_radius = m_display->toolRadius();
  cml::vector3d proxy_pos =  m_display->proxyPosition();
  cml::matrix33d proxy_rot = m_display->proxyOrientation();
  cml::vector3d HIP_pos = m_display->toolPosition();
  cml::matrix33d HIP_rot = m_display->toolOrientation();

  if(m_device_info.m_sensedRotation)
  {
    object_manipulator::shapes::Arrow arrow;
    arrow.dims = tf::Vector3(2*proxy_radius, proxy_radius/2, proxy_radius/2);
    arrow.frame.setRotation(cml_tools::cmlMatrixToTFQuaternion(proxy_rot));
    arrow.frame.setOrigin(cml_tools::cmlVectorToTF(proxy_pos));
    arrow.header.stamp = time_now;
    arrow.header.frame_id = m_display_frame.c_str();
    object_manipulator::drawArrow(pub_marker_, arrow, "proxy", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 0.3, 0.7, 1.0));

    arrow.dims = tf::Vector3(2*proxy_radius, proxy_radius/2, proxy_radius/2);
    arrow.frame.setRotation(cml_tools::cmlMatrixToTFQuaternion(HIP_rot));
    arrow.frame.setOrigin(cml_tools::cmlVectorToTF(HIP_pos));
    arrow.header.stamp = time_now;
    arrow.header.frame_id = m_display_frame.c_str();
    object_manipulator::drawArrow(pub_marker_, arrow, "HIP", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 0.4, 0.2, 0.6));
  }
  else
  {
//      object_manipulator::shapes::Sphere sphere;
//      sphere.dims = tf::Vector3(2*proxy_radius, 2*proxy_radius, 2*proxy_radius);
//      sphere.frame = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(pos.x, pos.y, pos.z));
//      sphere.header.frame_id = m_display_frame.c_str();
//      sphere.header.stamp = time_now;
//      object_manipulator::drawSphere(pub_marker_, sphere, "proxy", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 0.3, 0.7, 1.0));
//
//      //object_manipulator::shapes::Sphere sphere;
//      sphere.dims = tf::Vector3(1.9*proxy_radius, 1.9*proxy_radius, 1.9*proxy_radius);
//      sphere.frame = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(HIP.x, HIP.y, HIP.z));
//      sphere.header.frame_id = m_display_frame.c_str();
//      sphere.header.stamp = time_now;
//      object_manipulator::drawSphere(pub_marker_, sphere, "HIP", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 0.4, 0.2, 0.6));
  }

  if(!ros::ok())
  {
   MyHapticsThread *hthread = MyHapticsThread::instance();
   hthread->quit();
  }

}
