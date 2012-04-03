

#include "MyHapticsWidget.h"
#include "cml_conversions.h"

#include "MyHapticsThread.h"

#include <Haptics/PointShellIsosurface.h>
#include <Common/PointSampler.h>

const float proxy_mesh_scale = 1.0;

std::string mesh_basename = "coarse_gripper_10cm";
std::string mesh_stl = mesh_basename + ".stl";
std::string mesh_obj = mesh_basename + ".obj";
std::string absolute_path = "/u/aleeper/ros/jks-ros-pkg/haptic_ghosted_gripper/meshes/";
std::string package_path = "package://haptic_ghosted_gripper/meshes/";

  //! The constructor
HapticGhostedGripper::HapticGhostedGripper(bool use_haptics):
    nh_("/"), pnh_("~"),
    active_(false),
    m_display_frame("/torso_lift_link"),
    m_clicking(false),
    m_clickExternal(false)
{
  ROS_INFO("Constructing HapticGhostedGripper!");
  // Marker topics
  pub_marker_ = pnh_.advertise<visualization_msgs::Marker>("/markers", 100);
  pub_marker_array_ = pnh_.advertise<visualization_msgs::MarkerArray>("/markers_array", 1);

  // String status topic
  pub_status_ = pnh_.advertise<std_msgs::String>("servo_status", 10);

  // Topic for most recently selected device pose
  pub_selected_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("selected_pose", 1);

  // Topic for the proxy pose
  pub_proxy_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("proxy_pose", 1);

  // Create timer callback for auto-running of algorithm
  update_timer_ = nh_.createTimer(ros::Duration(0.03333), boost::bind(&HapticGhostedGripper::displayCallback, this));

  pnh_.param<double>("workspace_scale", workspace_scale_, 2.25);
  pnh_.param<std::string>("mesh_basename", mesh_basename, "coarse_gripper_10cm");
  mesh_stl = mesh_basename + ".stl";
  mesh_obj = mesh_basename + ".obj";
  pnh_.param<std::string>("absolute_path", absolute_path, "/u/aleeper/ros/jks-ros-pkg/haptic_ghosted_gripper/meshes/");
  pnh_.param<std::string>("package_path", package_path, "package://haptic_ghosted_gripper/meshes/");

  // Set up the chai world and device
  initializeHaptics();

  // This server is for the user to adjust the algorithm weights
  dyn_cb = boost::bind( &HapticGhostedGripper::dynamicCallback, this, _1, _2 );
  dyn_srv.setCallback(dyn_cb);

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

void HapticGhostedGripper::loadPointShell(const std::string &location, float scale)
{
    PointShellIsosurface *surface = dynamic_cast<PointShellIsosurface *>(m_isosurface);
    if (surface == 0) return;

    ROS_INFO("Loading pointshell from: %s", location.c_str());
//    string directory = location.toStdString();
    MeshGLM mesh("pointshell");
    if(!mesh.loadFromFile(location))
      ROS_ERROR("Failed to load object file from location: %s!", location.c_str());

    // use the mesh vertices as a point shell on the haptic isosurface
    surface->setPointShell(mesh.vertexPointer(), mesh.numVertices(), scale);
    ROS_INFO("Succeeded loading pointshell!");
}


void HapticGhostedGripper::initializeHaptics()
{
    ROS_INFO("Creating connection to device...");

    // create a haptic device handler and initialize
    m_handler = new cHapticDeviceHandler();
    ROS_INFO("Found %d attached devices!", m_handler->getNumDevices());
    cGenericHapticDevice* hapticDevice;
    m_handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    if (hapticDevice)
    {
        m_device_info = hapticDevice->getSpecifications();
        printDeviceSpecs(m_device_info);
    }
    hapticDevice->open();
    cVector3d force(0,0,0);
    hapticDevice->setForce(force);

    // create a haptics scene with all the haptic displays in it
    m_scene = new HapticScene();

    if (hapticDevice)
    {
        HapticDisplay *display = m_scene->addHapticDisplay(hapticDevice);
        m_display = display; //.append(display);

//        // center the device at (.5, .5, .5) where the volume will be
        cml::matrix44f_c center, orient = cml::identity_4x4();
        cml::matrix_translation(center, 0.f, 0.f, 0.f);
        //cml::matrix_set_basis_vectors(orient, -cml::x_axis_3D(), -cml::y_axis_3D(), cml::z_axis_3D());
        display->setTransform(orient * center);

        display->setToolRadius(config_.tool_radius);
        m_display->setWorkspaceRadius(workspace_scale_*display->deviceWorkspace());

        // TODO: increase stiffness when speed problem is solved
        double k = display->stiffness();
        display->setStiffness(0.3 * k / workspace_scale_);
        k = display->torsionalStiffness();
        display->setTorsionalStiffness(0.1*k);
    }
    cml::matrix33d perm;
    perm.set(-1.0,  0.0, 0.0,
              0.0, -1.0, 0.0,
              0.0,  0.0, 1.0);
    m_display->setPermutation(perm);


//    // save obj models to disk so we can load them back
//    QString location = expandModelResources();
//    string directory = location.toStdString();
//    m_drill = new MeshGLM("drill", directory + "/drill.obj",
//                          MeshGLM::k_useTexture | MeshGLM::k_useMaterial);
//    m_burr = new MeshGLM("burr", directory + "/burr1.obj",
//                         MeshGLM::k_useMaterial);


    m_sampler = new PointSampler();
    PointSampler *ps = dynamic_cast<PointSampler*>(m_sampler);
    ps->setRadiusMultiplier(2.5);
//    ps->createSphericalShell(0.1, 0.1, -M_PI/2.5, M_PI/3, -M_PI, M_PI, config_.synthetic_noise);
//    ps->applyLastCloud();

    //std::string location = "/u/aleeper/ros/jks-ros-pkg/haptic_ghosted_gripper/meshes/pointshell.obj";
    std::string location = absolute_path + mesh_obj;
    //m_mesh = new MeshGLM("drill", location, MeshGLM::k_useTexture | MeshGLM::k_useMaterial);


    // add a haptic isosurface to the scene
    //m_isosurface = new HapticIsosurface(m_sampler, 0.5);
    m_isosurface = new PointShellIsosurface(m_sampler, 0.5);
    ROS_INFO("Loading gripper model for haptic computations.");
    loadPointShell(location, proxy_mesh_scale);
    m_scene->addNode(m_isosurface);

    // clean up the temporary model files from the disk
    //removeModelResources(location);

    // add the scene to the haptics thread and select it for rendering
    MyHapticsThread *hthread = MyHapticsThread::instance();
    m_sceneIndex = hthread->addScene(m_scene);
    hthread->selectScene(m_sceneIndex);
//    if(!hthread->resume())
//    {
//      ROS_ERROR("Failed to resume haptics thread...something is very wrong!");
//    }
}

void HapticGhostedGripper::resumeHaptics()
{
  MyHapticsThread *hthread = MyHapticsThread::instance();
  if(!hthread->resume())
  {
    ROS_ERROR("Failed to resume haptics thread...something is very wrong!");
    return;
  }
  active_ = true;
}

void HapticGhostedGripper::pauseHaptics()
{
  MyHapticsThread *hthread = MyHapticsThread::instance();
  if(!hthread->pause())
  {
    ROS_WARN("Haptics thread already paused... are we doing something wrong? :)");
  }
  active_ = false;
}


void HapticGhostedGripper::loadPointCloud(pcl::PointCloud<PointT>::Ptr &cloud)
{
  PointSampler *ps = dynamic_cast<PointSampler*>(m_sampler);
  pauseHaptics();
  m_isosurface->reset();
  ps->createFromCloud(*cloud);
  ps->applyLastCloud();
  resumeHaptics();
}

bool HapticGhostedGripper::checkForButtonClick()
{
  // detect a rising edge on the click button
  if (!m_clicking && (m_display->buttonState(k_clickButton) || m_clickExternal))
  {
      m_clicking = true;
      return true;
  }
  // detect a falling edge on the click button
  else if (m_clicking && !(m_display->buttonState(k_clickButton) || m_clickExternal))
  {
      m_clicking = false;
  }
  return false;
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
      MyHapticsThread *hthread = MyHapticsThread::instance();
      sprintf(status_string, "Haptic rate: %4d Graphics Rate: %.3f",
              hthread->m_hapticFPS, fps_estimate);
      //ROS_INFO_STREAM(status_string);
      msg.data = status_string;
      pub_status_.publish(msg);

      PointShellIsosurface* surf = dynamic_cast<PointShellIsosurface*>(m_isosurface);
      size_t constraints = 0;
      if(surf) constraints = surf->getNumContactPoints();
      cml::vector3d device_pos =  m_display->devicePosition();
      sprintf(status_string, "HIP pos: % .2f % .2f % .2f  Constraints: %d",
              device_pos[0], device_pos[1], device_pos[2],
              constraints);
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



  if(checkForButtonClick())
  {
    pub_selected_pose_.publish(cml_tools::getPoseStamped(proxy_pos, proxy_rot, m_display_frame));
  }
  pub_proxy_pose_.publish(cml_tools::getPoseStamped(proxy_pos, proxy_rot, m_display_frame));

  //float intensity = m_sampler->intensityAt(HIP_pos);
//  cml::vector3f gradient = m_sampler->gradientAt(HIP_pos);
  //float intensity = m_isosurface->surface(HIP_pos);
  //cml::vector3f gradient = m_isosurface->gradient(HIP_pos);

  //ROS_INFO("intensity at (%.2f, %.2f, %.2f): %.2f", HIP_pos[0], HIP_pos[1], HIP_pos[2], intensity);

  if(m_device_info.m_sensedRotation)
  {
    object_manipulator::shapes::Arrow arrow;
//    arrow.dims = 10*tf::Vector3(2*proxy_radius, proxy_radius, proxy_radius);
//    arrow.frame.setRotation(cml_tools::cmlMatrixToTFQuaternion(proxy_rot));
//    arrow.frame.setOrigin(cml_tools::cmlVectorToTF(proxy_pos));
//    arrow.header.stamp = time_now;
//    arrow.header.frame_id = m_display_frame.c_str();
//    object_manipulator::drawArrow(pub_marker_, arrow, "proxy", 0, ros::Duration(), object_manipulator::msg::createColorMsg(0.2, 1.0, 0.7, 1.0));

    object_manipulator::shapes::Mesh mesh;
    mesh.frame.setRotation(cml_tools::cmlMatrixToTFQuaternion(proxy_rot));
    mesh.frame.setOrigin(cml_tools::cmlVectorToTF(proxy_pos));
    mesh.header.stamp = time_now;
    mesh.header.frame_id = m_display_frame.c_str();
    mesh.dims = tf::Vector3(proxy_mesh_scale, proxy_mesh_scale, proxy_mesh_scale);
    mesh.mesh_resource = package_path + mesh_stl;
    mesh.use_embedded_materials = false;
    object_manipulator::drawMesh(pub_marker_, mesh, "proxy", 0, ros::Duration(), object_manipulator::msg::createColorMsg(0.1, 1.0, 0.4, 0.9), !active_);

    object_manipulator::shapes::Sphere sphere;
    sphere.dims = tf::Vector3(2*proxy_radius, 2*proxy_radius, 2*proxy_radius);
    sphere.frame = tf::Transform(tf::Quaternion(0,0,0,1), cml_tools::cmlVectorToTF(proxy_pos));
    sphere.header.frame_id = m_display_frame.c_str();
    sphere.header.stamp = time_now;
    object_manipulator::drawSphere(pub_marker_, sphere, "proxy_point", 0, ros::Duration(), object_manipulator::msg::createColorMsg(0.2, 1.0, 0.2, 1.0), !active_);

//    arrow.dims = 10*tf::Vector3(2*proxy_radius, proxy_radius, proxy_radius);
//    arrow.frame.setRotation(cml_tools::cmlMatrixToTFQuaternion(HIP_rot));
//    arrow.frame.setOrigin(cml_tools::cmlVectorToTF(HIP_pos));
//    arrow.header.stamp = time_now;
//    arrow.header.frame_id = m_display_frame.c_str();
//    object_manipulator::drawArrow(pub_marker_, arrow, "HIP", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 0.4, 0.2, 0.6));

    mesh.frame.setRotation(cml_tools::cmlMatrixToTFQuaternion(HIP_rot));
    mesh.frame.setOrigin(cml_tools::cmlVectorToTF(HIP_pos));
    mesh.header.stamp = time_now;
    mesh.header.frame_id = m_display_frame.c_str();
    float device_scale = 0.98*proxy_mesh_scale;
    mesh.dims = tf::Vector3(device_scale, device_scale, device_scale);
    //mesh.mesh_resource = "package://haptic_ghosted_gripper/meshes/drill.dae";
    //mesh.mesh_resource = "package://haptic_ghosted_gripper/meshes/whole_gripper.stl";
    mesh.mesh_resource = package_path + mesh_stl;
    mesh.use_embedded_materials = false;
    object_manipulator::drawMesh(pub_marker_, mesh, "HIP", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 0.2, 0.2, 0.5), !active_);


    sphere.dims = tf::Vector3(2*proxy_radius, 2*proxy_radius, 2*proxy_radius);
    sphere.frame = tf::Transform(tf::Quaternion(0,0,0,1), cml_tools::cmlVectorToTF(HIP_pos));
    sphere.header.frame_id = m_display_frame.c_str();
    sphere.header.stamp = time_now;
    object_manipulator::drawSphere(pub_marker_, sphere, "HIP_point", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 0.2, 0.2, 0.5), !active_);
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

//  // ----------------------------------
//  // Update tangent plane visualization
//  object_manipulator::shapes::Cylinder box;
//  tf::Vector3 normal = cml_tools::cmlVectorToTF(gradient).normalized();
//  tf::Vector3 rotX = normal.cross(tf::Vector3(0,0,1)).normalized();
//  tf::Vector3 rotY = normal.cross(rotX).normalized();
//  tf::Quaternion quat;
//  btMatrix3x3 mat;
//  setBTMatrixColumns(mat, rotX, rotY, normal);
//  mat.getRotation(quat);
//  box.frame.setRotation(quat);
//  box.frame.setOrigin( cml_tools::cmlVectorToTF(proxy_pos)); // 0.5*proxy_radius*box.frame.getBasis().getColumn(2));
//  box.dims = tf::Vector3(5*proxy_radius, 5*proxy_radius, 0.0015);
//  box.header.frame_id = "/tool_frame";
//  box.header.stamp = time_now;
//  if(m_display->lastForce().length() > 0.01){
//    object_manipulator::drawCylinder(pub_marker_, box, "tPlane", 0, ros::Duration(), object_manipulator::msg::createColorMsg(0.2, 0.6, 1.0, 0.8), false);
//  }
//  else
//    object_manipulator::drawCylinder(pub_marker_, box, "tPlane", 0, ros::Duration(), object_manipulator::msg::createColorMsg(0.2, 0.6, 1.0, 0.8), true);
//
//  object_manipulator::shapes::Arrow arrow;
//  arrow.dims = tf::Vector3(0.03, 0.03, sqrt(gradient.length())/100.f);
//  setBTMatrixColumns(mat, normal, rotX, rotY);
//  mat.getRotation(quat);
//  arrow.frame.setRotation(quat);
//  arrow.frame.setOrigin(cml_tools::cmlVectorToTF(HIP_pos));
//  arrow.header.stamp = time_now;
//  arrow.header.frame_id = m_display_frame.c_str();
//  object_manipulator::drawArrow(pub_marker_, arrow, "gradient", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 1.0, 1.0, 1.0));

  static int count = 0;
  count++;
  if(count > 6){
    count = 0;
    publishPointCloud();
  }


  if(!ros::ok())
  {
   MyHapticsThread *hthread = MyHapticsThread::instance();
   hthread->quit();
  }
}


void HapticGhostedGripper::publishPointCloud()
{
  ros::Time now = ros::Time::now();

  boost::mutex::scoped_lock lock(mutex_);
  if(config_.publish_cloud)
  {
    PointSampler *sampler = dynamic_cast<PointSampler*>(m_sampler);
    if(!sampler) return;

    pcl::PointCloud<PointT>::Ptr last_points(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr last_normals(new pcl::PointCloud<pcl::Normal>());
    sampler->getPointCloud(0, last_points);
    sampler->getNormalCloud(0, last_normals);

    if(last_normals->points.size() == last_points->points.size())
    {
      visualization_msgs::Marker marker;
      marker.header = last_points->header;
      marker.ns = "cloud";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE_LIST; // CUBE, SPHERE, ARROW, CYLINDER
      marker.action = false?((int32_t)visualization_msgs::Marker::DELETE):((int32_t)visualization_msgs::Marker::ADD);
      marker.lifetime = ros::Duration();
      float scale = sampler->getActiveRadius()/2.0f;
      //ROS_INFO("Active radius is %.3f m ", sampler->getActiveRadius() );
      marker.scale = object_manipulator::msg::createVector3Msg(scale, scale, scale);
      marker.color = object_manipulator::msg::createColorMsg(0.5, 0.5, 0.5,1.0);

      float angle = now.toSec(); //time_now.toSec();
      //angle = M_PI/180.0 * 45.0; //config_.light_angle;
      tf::Vector3 light_source = tf::Vector3(0.5*cos(angle), 0.5*sin(angle), 0.3);
      object_manipulator::shapes::Sphere sphere;
      sphere.dims = tf::Vector3(0.02, 0.02, 0.02);
      sphere.frame = tf::Transform(tf::Quaternion(0,0,0,1), light_source);
      sphere.header.frame_id = m_display_frame;
      sphere.header.stamp = marker.header.stamp;
      object_manipulator::drawSphere(pub_marker_, sphere, "light", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 1.0, 1.0, 0.5));

      for(size_t i = 0; i < last_points->points.size(); i++)
      {
        const PointT &pt = last_points->points[i];
        const pcl::Normal &nl = last_normals->points[i];
        tf::Vector3 point = tf::Vector3(pt.x, pt.y, pt.z);
        tf::Vector3 N_vec = tf::Vector3(nl.normal[0], nl.normal[1], nl.normal[2]).normalized();
        tf::Vector3 L_vec = (light_source - point).normalized();
        tf::Vector3 color = fabs(L_vec.dot(N_vec))*tf::Vector3(1.0, 1.0, 1.0) + tf::Vector3(0.2, 0.2, 0.2);
        //tf::Vector3 color = tf::Vector3(pt.x*pt.x, pt.y*pt.y, 1.0);
        marker.colors.push_back(object_manipulator::msg::createColorMsg(color.x(), color.y(), color.z(), 1.0));;
        marker.points.push_back(object_manipulator::msg::createPointMsg(pt.x, pt.y, pt.z));
      }
      ROS_DEBUG("Publishing marker cloud with %zd points!", marker.points.size());
      pub_marker_.publish(marker);
    }
//    sensor_msgs::PointCloud2 msg;
//    pcl::toROSMsg(*(last_points), msg);
//    msg.header.frame_id = "/base_link";
//    msg.header.stamp = now;
//    pub_cloud_.publish(msg);
  }
}

//! The dynamic reconfigure callback for setting algorithm params, etc.
void HapticGhostedGripper::dynamicCallback(Config &new_config, uint32_t id)
{
  // This info will get printed back to reconfigure gui
  char status[256] = "\0";

  switch(id){
  case(-1): // Init
    // If you are tempted to put anything here, it should probably go in the constructor
    break;

  case(0): // Connect
    printf("Reconfigure GUI connected to me!\n");
    new_config = config_;
    break;

  case(1):  // Object select
    //printf("Reconfigure GUI connected to me!\n");
//    config_.auto_threshold = new_config.auto_threshold;
//    object->updateShape(new_config.object_select);
    //new_object->updateShape(new_config.object_select);
    break;

  case(10): // auto-threshold
//    config_.auto_threshold = new_config.auto_threshold;
//    object->updateShape(new_config.object_select);


  default:  // Error condition
    break;
//    ROS_INFO("Dynamic reconfigure did something, variable %d.", id);

  }

  config_ = new_config;
  if(!m_sampler) return;
  PointSampler* sampler = dynamic_cast<PointSampler*>(m_sampler);
  if(!sampler) return;

  ROS_INFO("Dynamic reconfigure is setting parameters (incoming changed param has id %d).", id);
  sampler->setRadiusMultiplier(config_.radius_multiple);
  //sampler->setActiveRadius(config_.basis_radius);
  m_isosurface->setIsosurfaceValue(config_.meta_thresh);


//  if(new_config.auto_threshold)
//  {
////      new_config.meta_thresh = config_.meta_thresh;
//    new_config.basis_radius = config_.basis_radius;
//  }
//  config_ = new_config;
//  if(tool) tool->setRadius(config_.tool_radius);
//
//  new_config.status = status;
}
