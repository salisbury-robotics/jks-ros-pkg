/*
 * grasp_adjust_server
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Software License Agreement (BSD License)
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

///\author Adam Leeper
///\brief Node for haptic interaction with a point cloud


#include "chai3d.h"
#include "CThreadWithArgument.h"

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

#include <boost/bind.hpp>
#include <sensor_msgs/PointCloud2.h>

//#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <hviz/HapticsConfig.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <stereo_msgs/DisparityImage.h>

#include <dynamic_reconfigure/server.h>

#include <object_manipulator/tools/shape_tools.h>


#include <iostream>
#include <fstream>
#include <queue>

#include "helpers.h"

#include "PointCloudObject.h"

#define PROF_ENABLED
#include <profiling/profiling.h>
PROF_DECLARE(TOTAL_TIMER)
PROF_DECLARE(FUNC_1)



//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

cWorld* world;
cCamera* camera;
cLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;
cGeneric3dofPointer* tool;
cHapticDeviceInfo device_info;

// the implicit surface object
PointCloudObject* object = 0; //, *new_object = 0;

// label to show estimate of haptic update rate
cLabel* rateLabel;
double rateEstimate = 0;
double graphicRateEstimate = 0;
bool simulationRunning = false;
bool simulationFinished = false;

void hapticsDispatch(void *instance);

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------



class HapticNode{

  pcl::PointCloud<PointT> last_cloud_, cloud_in_;

  ros::NodeHandle nh_, nh_pvt_;

  // The input cloud
  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_disparity_1_;

  // A republished cloud for displaying the current haptic cloud
  ros::Publisher pub_cloud_, pub_marker_cloud_;

  // Publishers for markers (e.g. haptic tool)
  ros::Publisher pub_marker_, pub_marker_array_;

  // Publisher for string status data
  ros::Publisher pub_status_;

  // A timer callback
  ros::Timer update_timer_;
  ros::Timer slow_update_timer_;

  std::string input_topic_;
  std::string output_topic_;

  tf::TransformListener tfl_;
  tf::TransformBroadcaster tfb_;

  //bool refresh_cloud;

  ros::Time now_;

  bool got_first_cloud_;

  boost::mutex mutex_;

  // ***** Dynamic reconfigure stuff *****
  typedef hviz::HapticsConfig Config;
  Config config_;
  dynamic_reconfigure::Server<Config>                dyn_srv;
  dynamic_reconfigure::Server<Config>::CallbackType  dyn_cb;


public:
  /*!
  * \brief Default contructor.
  *
  * This description is displayed lower in the doxygen as an extended description along with
  * the above brief description.
  */
  HapticNode(char* argv[]):
      nh_("/")
      , nh_pvt_("~")
      , input_topic_("/cloud_in")
      , output_topic_("/cloud_out")
      //, dyn_srv(ros::NodeHandle("~config")),
  {
    got_first_cloud_ = false;
    now_ = ros::Time::now() - ros::Duration(1.0);

    // Clouds in and out
    sub_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>
                 (input_topic_, 1, boost::bind(&HapticNode::cloudCallback,this, _1));
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
    pub_marker_cloud_ = nh_.advertise<visualization_msgs::MarkerArray>("/marker_cloud_array", 1);

    // Disparity images in and out:
    sub_disparity_1_ = nh_.subscribe<stereo_msgs::DisparityImage>
                 ("/disparity_1", 1, boost::bind(&HapticNode::disparityCallback, this, _1));

    // Marker topics
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>("/markers", 100);
    pub_marker_array_ = nh_.advertise<visualization_msgs::MarkerArray>("/markers_array", 1);

    // String status topic
    pub_status_ = nh_pvt_.advertise<std_msgs::String>("status", 10);

    // Create timer callback for auto-running of algorithm
    update_timer_ = nh_.createTimer(ros::Duration(0.03333), boost::bind(&HapticNode::timerCallback, this));
    slow_update_timer_ = nh_.createTimer(ros::Duration(0.5), boost::bind(&HapticNode::slowTimerCallback, this));


    // This server is for the user to adjust the algorithm weights
    dyn_cb = boost::bind( &HapticNode::dynamicCallback, this, _1, _2 );
    dyn_srv.setCallback(dyn_cb);

    // Set up the chai world and device
    initializeHaptics();

    ROS_INFO("Finished constructor!");
  }

  void initializeHaptics()
  {
    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    // the color is defined by its (R,G,B) components.
    world->setBackgroundColor(0.2, 0.2, 0.2);

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and oriente the camera
    camera->set( cVector3d (5.0, 0.0, 0.0),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // enable higher quality rendering for transparent objects
    camera->enableMultipassTransparency(true);

    // create a light source and attach it to the camera
    light = new cLight(world);
    camera->addChild(light);                   // attach light to camera
    light->setEnabled(true);                  // enable light source
    light->setPos(cVector3d( 3.0, 1.5, 2.0));  // position the light source
    light->setDir(cVector3d(-2.0, 0.5, 1.0));  // define the direction of the light beam

    // create a label that shows the haptic loop update rate
    rateLabel = new cLabel();
    rateLabel->setPos(8, 8, 0);
    camera->m_front_2Dscene.addChild(rateLabel);


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    ROS_INFO("Creating connection to device...");
    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    cGenericHapticDevice* hapticDevice;

    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    if (hapticDevice)
    {
        device_info = hapticDevice->getSpecifications();
    }

    // create a 3D tool and add it to the world
    tool = new cGeneric3dofPointer(world);
    tool->setHapticDevice(hapticDevice);
    tool->start();
    float workspace_radius = 0.25;
    tool->setWorkspaceRadius(workspace_radius);
    tool->setRadius(0.01);
    printf("Device is %s\n", device_info.m_modelName.c_str());
    if( !device_info.m_modelName.compare("PHANTOM Omni"))
      tool->setPos(-workspace_radius/2.5, 0, 0);
    else
      tool->setPos(0, 0, 0);
    world->addChild(tool);
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();
    //ROS_INFO("Tool has workspace scale factor %lf", workspaceScaleFactor);
    double stiffnessMax = device_info.m_maxForceStiffness / workspaceScaleFactor;

    object = new PointCloudObject(world);
    object->addEffect(new cEffectSurface(object));
    object->m_material.setStiffness(0.7 * stiffnessMax);
    object->m_material.setDynamicFriction(0.2);
    object->setUseVertexColors(true, false);
    object->useDisplayList(false, true);
    object->useVertexArrays(true, true);
    object->m_tool = tool;
    object->m_tag = 0;
    object->setAsGhost(false);
    object->m_configPtr = &config_;

//    new_object = new PointCloudObject(world);
//    new_object->addEffect(new cEffectSurface(object));
//    new_object->m_material.setStiffness(0.9 * stiffnessMax);
//    new_object->m_material.setDynamicFriction(0.2);
//    new_object->setUseVertexColors(true, false);
//    new_object->useDisplayList(false, true);
//    new_object->useVertexArrays(true, true);
//    new_object->m_tool = tool;
//    new_object->m_tag = 1;
//    new_object->setAsGhost(true);
//    new_object->m_configPtr = &config_;

    world->addChild(object);
//    world->addChild(new_object);

    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // simulation in now running
    simulationRunning = true;

    // create a thread which starts the main haptics rendering loop
    cThreadWithArgument* hapticsThread = new cThreadWithArgument();
    hapticsThread->set(hapticsDispatch, this, CHAI_THREAD_PRIORITY_HAPTICS);

  }

  private:

  /*!
  * \brief The timer callback sends graphical output to rviz.
  *
  * This description is displayed lower in the doxygen as an extended description along with
  * the above brief description.
  */
  void timerCallback()
  {
    if(!tool) return;

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
        graphicRateEstimate = counter;
        counter = 0;
        pclock.start(true);
        std_msgs::String msg;
        char status_string[256];
        sprintf(status_string, "Haptic rate: %.3f Graphics Rate: %.3f",
                rateEstimate, graphicRateEstimate);
        msg.data = status_string;
        pub_status_.publish(msg);
    }

    // Transmit the visualizations of the tool/proxy
    float proxy_radius = config_.tool_radius;
    cVector3d pos =  object->m_interactionProjectedPoint; //tool->getDeviceGlobalPos();
    cVector3d HIP = tool->getDeviceGlobalPos();
    cMatrix3d tool_rotation = tool->m_deviceGlobalRot;

    if(false && device_info.m_sensedRotation)
    {
      object_manipulator::shapes::Mesh mesh;
      mesh.dims = tf::Vector3(0.5, 0.5, 0.5);
      mesh.frame.setRotation(chai_tools::cMatrixToTFQuaternion(tool_rotation));
      mesh.frame.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));
      mesh.header.stamp = time_now;
      mesh.header.frame_id = "/tool_frame";
      mesh.use_embedded_materials = true;
      mesh.mesh_resource = std::string("package://pr2_description/meshes/gripper_v0/gripper_palm.dae");
//      std::string proximal_finger_string("package://pr2_description/meshes/gripper_v0/l_finger.dae");
//      std::string distal_finger_string("package://pr2_description/meshes/gripper_v0/l_finger_tip.dae");
      object_manipulator::drawMesh(pub_marker_, mesh, "gripper", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 0.3, 0.7, 1.0));
    }
    else
    {
      object_manipulator::shapes::Sphere sphere;
      sphere.dims = tf::Vector3(2*proxy_radius, 2*proxy_radius, 2*proxy_radius);
      sphere.frame = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(pos.x, pos.y, pos.z));
      sphere.header.frame_id = "/tool_frame";
      sphere.header.stamp = time_now;
      object_manipulator::drawSphere(pub_marker_, sphere, "proxy", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 0.3, 0.7, 1.0));

      //object_manipulator::shapes::Sphere sphere;
      sphere.dims = tf::Vector3(1.9*proxy_radius, 1.9*proxy_radius, 1.9*proxy_radius);
      sphere.frame = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(HIP.x, HIP.y, HIP.z));
      sphere.header.frame_id = "/tool_frame";
      sphere.header.stamp = time_now;
      object_manipulator::drawSphere(pub_marker_, sphere, "HIP", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 0.0, 0.0, 0.6));
    }


    object_manipulator::shapes::Cylinder box;
    tf::Quaternion quat = chai_tools::cMatrixToTFQuaternion(object->tPlane->getRot());
    box.frame.setRotation(quat);
    box.frame.setOrigin(chai_tools::cVectorToTF(object->tPlane->getPos()) - 0.5*proxy_radius*box.frame.getBasis().getColumn(2));
    box.dims = tf::Vector3(5*proxy_radius, 5*proxy_radius, 0.0015);
    box.header.frame_id = "/tool_frame";
    box.header.stamp = time_now;
    if(object->m_interactionInside){
      object_manipulator::drawCylinder(pub_marker_, box, "tPlane", 0, ros::Duration(), object_manipulator::msg::createColorMsg(0.2, 0.6, 1.0, 0.8), false);
    }
    else
      object_manipulator::drawCylinder(pub_marker_, box, "tPlane", 0, ros::Duration(), object_manipulator::msg::createColorMsg(0.2, 0.6, 1.0, 0.8), true);


    boost::mutex::scoped_lock lock(mutex_);
    if(config_.publish_cloud)
    {

      if(object->last_normals->points.size() == object->last_points->points.size())
      {
        visualization_msgs::Marker marker;
        marker.header = object->last_points->header;
        marker.ns = "cloud";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST; // CUBE, SPHERE, ARROW, CYLINDER
        marker.action = false?((int32_t)visualization_msgs::Marker::DELETE):((int32_t)visualization_msgs::Marker::ADD);
        marker.lifetime = ros::Duration();
        float scale = object->m_active_radius/2;
        marker.scale = object_manipulator::msg::createVector3Msg(scale, scale, scale);
        marker.color = object_manipulator::msg::createColorMsg(0.5, 0.5, 0.5,1.0);

        float angle = time_now.toSec();
        angle = config_.light_angle * M_PI/180.0;
        tf::Vector3 light_source = tf::Vector3(0.1*cos(angle), 0.1*sin(angle), 0.3);
        object_manipulator::shapes::Sphere sphere;
        sphere.dims = tf::Vector3(0.02, 0.02, 0.02);
        sphere.frame = tf::Transform(tf::Quaternion(0,0,0,1), light_source);
        sphere.header.frame_id = "/tool_frame";
        sphere.header.stamp = marker.header.stamp;
        object_manipulator::drawSphere(pub_marker_, sphere, "light", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 1.0, 1.0, 0.5));



        for(int i = 0; i < object->last_points->points.size(); i++)
        {
          const PointT &pt = object->last_points->points[i];
          const pcl::Normal &nl = object->last_normals->points[i];
          tf::Vector3 point = tf::Vector3(pt.x, pt.y, pt.z);
          tf::Vector3 N_vec = tf::Vector3(nl.normal[0], nl.normal[1], nl.normal[2]).normalized();
          tf::Vector3 L_vec = (light_source - point).normalized();
          tf::Vector3 color = fabs(L_vec.dot(N_vec))*tf::Vector3(1.0, 1.0, 1.0) + tf::Vector3(0.2, 0.2, 0.2);
          //tf::Vector3 color = tf::Vector3(pt.x*pt.x, pt.y*pt.y, 1.0);
          marker.colors.push_back(object_manipulator::msg::createColorMsg(color.x(), color.y(), color.z(), 1.0));;
          marker.points.push_back(object_manipulator::msg::createPointMsg(pt.x, pt.y, pt.z));
        }
        //ROS_INFO("Publishing marker cloud with %d points!", marker.points.size());
        pub_marker_.publish(marker);
      }
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(*(object->last_points), msg);
      msg.header.frame_id = "/tool_frame";
      msg.header.stamp = time_now;
      pub_cloud_.publish(msg);
    }
  }

void slowTimerCallback()
{
//  visualization_msgs::Marker marker;
//  marker.header = object->m_cloud_points->header;
//  marker.ns = "cloud";
//  marker.id = 0;
//  marker.type = visualization_msgs::Marker::SPHERE_LIST; // CUBE, SPHERE, ARROW, CYLINDER
//  marker.action = false?((int32_t)visualization_msgs::Marker::DELETE):((int32_t)visualization_msgs::Marker::ADD);
//  //marker.lifetime = ros::Duration();
//  float scale = config_.basis_radius/2;
//  marker.scale = object_manipulator::msg::createVector3Msg(scale, scale, scale);
//  marker.color = object_manipulator::msg::createColorMsg(1.0, 0.0, 0.0, 1.0);
//
//  tf::Vector3 light_source = tf::Vector3(0.1, 0.1, 0.3);
//  object_manipulator::shapes::Sphere sphere;
//  sphere.dims = tf::Vector3(0.1, 0.1, 0.1);
//  sphere.frame = tf::Transform(tf::Quaternion(0,0,0,1), light_source);
//  sphere.header.frame_id = "/tool_frame";
//  sphere.header.stamp = marker.header.stamp;
//  object_manipulator::drawSphere(pub_marker_, sphere, "light", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 1.0, 0.0, 1.0));
//
//  for(int i = 0; i < object->m_cloud_points->points.size(); i++)
//  {
//    const PointT &pt = object->m_cloud_points->points[i];
//    const pcl::Normal &nl = object->m_cloud_normals->points[i];
//    tf::Vector3 point = tf::Vector3(pt.x, pt.y, pt.z);
//    tf::Vector3 N_vec = tf::Vector3(nl.normal[0], nl.normal[1], nl.normal[2]);
//    tf::Vector3 L_vec = (light_source - point).normalized();
//    tf::Vector3 color = fabs(L_vec.dot(N_vec))*tf::Vector3(0.0, 0.0, 1.0) + tf::Vector3(1.0, 0.0, 0.0);
////    tf::Vector3 color = tf::Vector3(1.0, 0.2, 0.2);
//    marker.colors.push_back(object_manipulator::msg::createColorMsg(color.x(), color.y(), color.z(), 1.0));
//    marker.points.push_back(object_manipulator::msg::createPointMsg(pt.x, pt.y, pt.z));
//  }
//  ROS_INFO("Publishing marker cloud with %d points!", marker.points.size());
//  pub_marker_.publish(marker);

}

  /*!
  * \brief The haptic update loop.
  *
  * This description is displayed lower in the doxygen as an extended description along with
  * the above brief description.
  */
  friend void hapticsDispatch(void *);
  void updateHaptics(void)
  {
      // a clock to estimate the haptic simulation loop update rate
      cPrecisionClock pclock;
      pclock.setTimeoutPeriodSeconds(1.0);
      pclock.start(true);
      int counter = 0, loop_counter = 0;

      // main haptic simulation loop
      while(simulationRunning)
      {
        usleep(config_.haptics_sleep);

        if(object->m_refresh_cloud)
        {
//          // Copy over state info from old object
//          new_object->m_interactionInside = object->m_interactionInside;
//          new_object->m_interactionProjectedPoint = object->m_interactionProjectedPoint;
//          new_object->tPlane = object->tPlane;
//          //new_object->tPlaneNormal = object->tPlaneNormal;
//          new_object->m_shape = object->m_shape;
//          new_object->m_showTangent = object->m_showTangent;
//
//          PointCloudObject *temp = object;
//          object = new_object;
//          new_object = temp;
//
//          object->setAsGhost(false);
//          new_object->setAsGhost(true);

          ros::WallTime begin = ros::WallTime::now();
          object->applyLastCloud();
          ROS_DEBUG_NAMED("time", "Applying cloud took %.3lf ms", (ros::WallTime::now() - begin).toSec()*1000.0);
          //refresh_cloud = false;
        }

        object->m_material.setDynamicFriction(config_.dynamic_friction);
        object->scaleZ = config_.scaleZ;
        object->m_cloudType = config_.cloud_mode;
        object->m_basisType = config_.basis_function;

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updatePose();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to device
        tool->applyForces();

        bool buttonState = false;
        tool->getHapticDevice()->getUserSwitch(0, buttonState);

        // This is how we move the workspace!
        if(buttonState)
        {
          float translate_factor = 0.001;
          cVector3d localPos = tool->getDeviceLocalPos();
          if(sqrt(localPos.x*localPos.x + localPos.y*localPos.y) > 1.5){
            camera->translate(translate_factor*cDot(localPos, cVector3d(1,0,0)),
                              translate_factor*cDot(localPos, cVector3d(0,1,0)),
                              0);

              //m_cam.strafeRight(0.0005*cDot(localPos, cVector3d(0,1,0)));
            //m_cam.moveForward(0.0005*cDot(localPos, cVector3d(-1,0,0)));
            tool->m_lastComputedGlobalForce += 2*cVector3d(-localPos.x, -localPos.y, 0);
            tool->applyForces();
          }
        }

        // estimate the refresh rate
        ++counter;
        if (pclock.timeoutOccurred()) {
            pclock.stop();
            rateEstimate = counter;
            counter = 0;
            pclock.start(true);
        }
      }

      // exit haptics thread
      simulationFinished = true;
  }


  /*!
  * \brief Callback for receiving a point cloud.
  *
  * This description is displayed lower in the doxygen as an extended description along with
  * the above brief description.
  */
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
  {

//    const std::string& publisher_name = event.getPublisherName();
//    ros::M_string connection_header = event.getConnectionHeader();
//    ros::Time receipt_time = event.getReceiptTime();
//    const std::string topic = connection_header["topic"];

//    const sensor_msgs::PointCloud2& input = event.getMessage();

    std::string topic = "none";

    int cloud_size = input->width*input->height;
    ROS_DEBUG_NAMED("haptics", "Got a cloud on topic %s in frame %s with %d points!",
               topic.c_str(),
               input->header.frame_id.c_str(),
               cloud_size);

    if(cloud_size == 0) return;

    pcl::fromROSMsg(*input, last_cloud_);
    
    if(last_cloud_.header.frame_id.compare("/tool_frame"))
    {
//        ROS_INFO("Transforming cloud with %d points from %s to %s.",
//                 (int)last_cloud_.points.size(), last_cloud_.header.frame_id.c_str(),
//                 "/world");
        if(!tfl_.waitForTransform("/tool_frame", last_cloud_.header.frame_id,
                                  last_cloud_.header.stamp, ros::Duration(2.0)) )
        {
            ROS_ERROR("Couldn't get transform for cloud, returning FAILURE!");
            return;
        }
        pcl_ros::transformPointCloud("/tool_frame", last_cloud_, last_cloud_, tfl_);
    }

    //ROS_INFO("m_shape is %d", m_shape);
    //if(new_object->m_shape == hviz::Haptics_CLOUD)
    {
      boost::mutex::scoped_lock lock(mutex_);
      if(object->m_shape == hviz::Haptics_CLOUD)
        object->createFromCloud(last_cloud_);
    }

    if(!got_first_cloud_){
      got_first_cloud_ = true;
      ROS_INFO("Got first cloud!");
    }

    static bool firstTime = true;
    static int counter = 0;
    static cPrecisionClock pclock;
    float sample_period = 2.0;

    if(firstTime) // start a clock to estimate the rate
    {
        pclock.setTimeoutPeriodSeconds(sample_period);
        pclock.start(true);
        firstTime = false;
    }

    // estimate the refresh rate and publish
    ++counter;
    if (pclock.timeoutOccurred()) {
        pclock.stop();
        float rate = counter/sample_period;
        counter = 0;
        pclock.start(true);
        std_msgs::String msg;
        char status_string[256];
        sprintf(status_string, "Cloud rate: %.3f",
                rate );
        msg.data = status_string;
        pub_status_.publish(msg);
    }

  }

  /*!
  * \brief Callback for receiving a point cloud.
  *
  * This description is displayed lower in the doxygen as an extended description along with
  * the above brief description.
  */
  void disparityCallback(const stereo_msgs::DisparityImageConstPtr &input)
  {

  }


  /*!
  * \brief Callback for setting algorithm params, such as weights, through dynamic reconfigure.
  *
  * This description is displayed lower in the doxygen as an extended description along with
  * the above brief description.
  */
  void dynamicCallback(Config &new_config, uint32_t id)
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
      config_.auto_threshold = new_config.auto_threshold;
      object->updateShape(new_config.object_select);
      //new_object->updateShape(new_config.object_select);
      break;

    case(10): // auto-threshold
      config_.auto_threshold = new_config.auto_threshold;
      object->updateShape(new_config.object_select);


    default:  // Error condition
      ROS_INFO("Dynamic reconfigure did something, variable %d.", id);

    }

    if(new_config.auto_threshold)
    {
//      new_config.meta_thresh = config_.meta_thresh;
      new_config.basis_radius = config_.basis_radius;
    }
    config_ = new_config;
    if(tool) tool->setRadius(config_.tool_radius);

    new_config.status = status;
  }

};  // End of class definition

void hapticsDispatch(void *instance)
{
    HapticNode *widget = reinterpret_cast<HapticNode *>(instance);
    if (widget) widget->updateHaptics();
}


/* ---[ */
int main (int argc, char* argv[])
{
  ros::init(argc, argv, "haptics_node");
  HapticNode haptics(argv);
  ros::MultiThreadedSpinner spinner(3); // Use 4 threads
  spinner.spin();

  return (0);
}
/* ]--- */
