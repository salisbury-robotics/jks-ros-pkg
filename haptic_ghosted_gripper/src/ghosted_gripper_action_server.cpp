/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

// author: Adam Leeper

// **** 10 ***** 20 ****** 30 ****** 40 ****** 50 ****** 60 ****** 70 ****** 80 ****** 90 ***** 100 ***** 110 ***** 120

#include <ros/ros.h>
#include <math.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <object_manipulator/tools/mechanism_interface.h>
#include <object_manipulator/tools/msg_helpers.h>

#include <boost/thread.hpp>
#include <tf/transform_listener.h>
#include <boost/thread/recursive_mutex.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <point_cloud_server/StoreCloudAction.h>

#include <pr2_im_msgs/TestGripperPoseAction.h>
#include <pr2_im_msgs/GetGripperPoseAction.h>

#include <sensor_msgs/point_cloud_conversion.h>

#include "eigen_conversions/eigen_msg.h"

#include "eigen3/Eigen/Geometry"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl_ros/transforms.h"
#include <pcl/filters/voxel_grid.h>

#include <household_objects_database_msgs/GetModelDescription.h>
#include <household_objects_database_msgs/GetModelMesh.h>

//#include "marker_helpers.h"

#include "cml_conversions.h"

#include "ghosted_gripper_action_server.h"

using namespace object_manipulator;
using namespace visualization_msgs;
using namespace interactive_markers;
using namespace pr2_im_msgs;



GhostedGripperActionServer::GhostedGripperActionServer(bool use_haptics) :
    object_cloud_(new pcl::PointCloud<PointT>()),
    active_(false),
    object_model_(false),
    nh_("/"),
    pnh_("~"),
    server_("haptic_ghosted_gripper", "server 1", false),
    tfl_(nh_),
    test_pose_client_("test_gripper_pose", true),
    cloud_server_client_("point_cloud_server_action", true),
    get_pose_name_(ros::this_node::getName()),
    get_pose_server_(nh_, get_pose_name_, false),
    haptic_interface_(use_haptics),
    use_haptics_(use_haptics)
{
  ROS_INFO( "pr2_ghosted_gripper IM server is running." );

  ros::Duration(1.0).sleep();
  pose_state_ = UNTESTED;
  gripper_angle_ = 0.541;
  initMenus();

  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = "/base_link";
  server_.insert(makeButtonBox( "test_box", ps, 0.01, false, false));

  proxy_pose_ = getDefaultPose();

  //pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_debug", 1);

  get_model_mesh_client_ = nh_.serviceClient<household_objects_database_msgs::GetModelMesh>("objects_database_node/get_model_mesh", false);

  spin_timer_ =  nh_.createTimer(ros::Duration(0.05), boost::bind( &GhostedGripperActionServer::spinOnce, this ) );

  sub_seed_ = nh_.subscribe<geometry_msgs::PoseStamped>("cloud_click_point", 1, boost::bind(&GhostedGripperActionServer::setSeed, this, _1));
  sub_selected_pose_ = pnh_.subscribe<geometry_msgs::PoseStamped>("selected_pose", 1, boost::bind(&GhostedGripperActionServer::setSelectedPose, this, _1));
  sub_proxy_pose_ = pnh_.subscribe<geometry_msgs::PoseStamped>("proxy_pose", 1, boost::bind(&GhostedGripperActionServer::setProxyPose, this, _1));
  sub_refresh_flag_ = nh_.subscribe<std_msgs::String>("refresh_flag", 1, boost::bind(&GhostedGripperActionServer::getAndLoadCloudSnapshot, this, _1));
  pnh_.param<double>("voxel_size", voxel_size_, 0.002);
//  pnh_.param<bool>("use_haptics", use_haptics_, false);
  pnh_.param<std::string>("haptic_frame_id", haptic_frame_id_, "/torso_lift_link");

//  haptic_interface_.initializeHaptics();

  //register the goal and feeback callbacks
  get_pose_server_.registerGoalCallback(    boost::bind(&GhostedGripperActionServer::goalCB, this));
  get_pose_server_.registerPreemptCallback( boost::bind(&GhostedGripperActionServer::preemptCB, this));

  get_pose_server_.start();
}

bool GhostedGripperActionServer::transformPoseToCommonFrame(const geometry_msgs::PoseStamped &ps,
                                                            geometry_msgs::PoseStamped &result)
{
  try
  {
    tfl_.waitForTransform(haptic_frame_id_, ps.header.frame_id, ps.header.stamp, ros::Duration(2.0));
    tfl_.transformPose(haptic_frame_id_, ps, result );
  }
  catch(...)
  {
    ROS_ERROR("TF could not transform from [%s] to [%s].", haptic_frame_id_.c_str(), ps.header.frame_id.c_str());
    return false;
  }
  return true;
}

void GhostedGripperActionServer::setSeed(const geometry_msgs::PoseStampedConstPtr &seed)
{
  if(!active_) return;
  geometry_msgs::PoseStamped ps;
  haptic_interface_.m_isosurface->reset();
  transformPoseToCommonFrame(*seed, ps);
  ROS_DEBUG("Setting seed.");
  if(!use_haptics_)
  {
    ROS_DEBUG_STREAM("Input seed was \n" << ps);
    tf::Pose pose;
    tf::poseMsgToTF(ps.pose, pose);
    tf::Quaternion q = pose.getRotation();
    btMatrix3x3 rot(q);
    btMatrix3x3 perm(  0, 0, 1,
                       0, 1, 0,
                      -1, 0, 0);
    (rot*perm).getRotation(q);
    //tf::quaternionTFToMsg(q, ps.pose.orientation);
    pose.setRotation(q);

    if(object_model_) pose = pose*tf::Transform(tf::Quaternion(tf::Vector3(0,1,0), M_PI/2.0), tf::Vector3(0,0,0)).inverse();

    tf::poseTFToMsg(pose, ps.pose);

    selected_pose_ = toWrist(ps);

    // Have to update the device pose
    geometry_msgs::PoseStamped ps = fromWrist(selected_pose_);
    haptic_interface_.m_display->setClutchedPose(cml_tools::vectorMsgToCML(ps.pose.position),
                                                 cml_tools::quaternionMsgToCML(ps.pose.orientation));
    haptic_interface_.m_isosurface->update(haptic_interface_.m_display);

    initMarkers();

    pose_state_ = UNTESTED;
    if(haptic_interface_.isReady())
    {
      testing_current_grasp_ = true;
      testPose(selected_pose_, gripper_opening_);
    }
  }
  else
  {
    setHapticDeviceTransform(ps);
  }
}

void GhostedGripperActionServer::setSelectedPose(const geometry_msgs::PoseStampedConstPtr &seed)
{
  if(!active_) return;
  if(!haptic_interface_.isReady()) return;
  if(!use_haptics_) return;

  ROS_DEBUG("Setting pose.");
  geometry_msgs::PoseStamped ps = *seed;
  ROS_DEBUG_STREAM("Input seed was \n" << ps);
  tf::Pose pose;
  tf::poseMsgToTF(ps.pose, pose);
  tf::Quaternion q = pose.getRotation();
  btMatrix3x3 rot(q);
  btMatrix3x3 perm( -1, 0, 0,
                     0,-1, 0,
                     0, 0, 1);
  (rot*perm).getRotation(q);
  //tf::quaternionTFToMsg(q, ps.pose.orientation);
  pose.setRotation(q);

  //if(object_model_) pose = pose*tf::Transform(tf::Quaternion(tf::Vector3(0,1,0), M_PI/2.0), tf::Vector3(0,0,0)).inverse();

  tf::poseTFToMsg(pose, ps.pose);
  //ps.header = seed->header;
  //ROS_INFO_STREAM("Processed seed before wrist offset was \n" << ps);

  selected_pose_ = toWrist(ps);

  pose_state_ = UNTESTED;
  initMarkers();
  testing_current_grasp_ = true;
  testPose(selected_pose_, gripper_opening_);
}

void GhostedGripperActionServer::setProxyPose(const geometry_msgs::PoseStampedConstPtr &seed)
{
  if(!active_) return;
  ROS_DEBUG("Setting pose.");
  geometry_msgs::PoseStamped ps = *seed;
  ROS_DEBUG_STREAM("Input seed was \n" << ps);
  if(use_haptics_)
  {
    tf::Pose pose;
    tf::poseMsgToTF(ps.pose, pose);
    tf::Quaternion q = pose.getRotation();
    btMatrix3x3 rot(q);
    btMatrix3x3 perm( -1, 0, 0,
                       0,-1, 0,
                       0, 0, 1);
    (rot*perm).getRotation(q);
    //tf::quaternionTFToMsg(q, ps.pose.orientation);
    pose.setRotation(q);

    tf::poseTFToMsg(pose, ps.pose);
  }

  proxy_pose_ = toWrist(ps);
  initProxyMarker();
}

geometry_msgs::PoseStamped GhostedGripperActionServer::getDefaultPose()
{
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = haptic_frame_id_;
  ps.header.stamp = ros::Time::now();
  if(use_haptics_) {
    ps.pose.position.x = 0.7;
    ps.pose.position.z = -0.25;
    ps.pose.orientation.y = -0.707;
    ps.pose.orientation.w =  0.707;
  }
  else
  {
    ps.pose.position.x = 0.5;
    ps.pose.position.z = -0.25;
    ps.pose.orientation.w =  1;
  }
  return ps;
}

//! Remove the markers.
void GhostedGripperActionServer::setIdle(){
  active_ = false;
  clearAllMarkers();
  //server_.erase("grasp_toggle");
}


//! Callback to accept a new action goal.
void GhostedGripperActionServer::goalCB()
{
  active_ = true;
  object_model_ = false;
  ROS_INFO("Ghosted gripper called");
  pr2_im_msgs::GetGripperPoseGoal goal = *get_pose_server_.acceptNewGoal();
  object_cloud_.reset( new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  if(use_haptics_)
    control_offset_.pose = object_manipulator::msg::createPoseMsg(tf::Pose(tf::Quaternion::getIdentity(), tf::Vector3(0.10,0,0)));
  else
    control_offset_.pose = object_manipulator::msg::createPoseMsg(tf::Pose(tf::Quaternion::getIdentity(), tf::Vector3(0.15,0,0)));

  if( !goal.object.cluster.points.empty() ||
      !goal.object.potential_models.empty() )
  {
    ROS_INFO("Goal object contains %d cluster and %d models.",
             !goal.object.cluster.points.empty(), (int)goal.object.potential_models.size() );
    // Something to draw...
    try
    {
      ROS_INFO("Converting to reference_frame...");
      mechanism_.convertGraspableObjectComponentsToFrame(goal.object, goal.object.reference_frame_id);

      bool use_cloud = goal.object.potential_models.empty();

      // Try to use object model, if there is one
      if(!use_cloud)
      {
        ROS_INFO("Goal contains object model; looking for model mesh...");
        // Connect to databse, get object mesh.
        arm_navigation_msgs::Shape mesh;
        if(!getModelMesh(goal.object.potential_models[0].model_id, mesh))
        {
          ROS_INFO("Unable to get database model, continuing with cluster.");
          use_cloud = true;
        }

        if(!use_cloud)
        {
          object_model_ = true;
          for(unsigned int i = 0; i < mesh.vertices.size(); i++)
          {
            PointT pt;
            pt.x = mesh.vertices[i].x;
            pt.y = mesh.vertices[i].y;
            pt.z = mesh.vertices[i].z;
            cloud->points.push_back(pt);
          }
          cloud->width = cloud->points.size();
          cloud->height = 1;
          cloud->is_dense = false;
          cloud->header.frame_id = goal.object.reference_frame_id;

          geometry_msgs::Pose &m = goal.object.potential_models[0].pose.pose;
          Eigen::Affine3f affine = Eigen::Translation3f(m.position.x,
                                                              m.position.y,
                                                              m.position.z) *
                                       Eigen::Quaternionf(m.orientation.w,
                                                          m.orientation.x,
                                                          m.orientation.y,
                                                          m.orientation.z);
          pcl::transformPointCloud(*cloud, *cloud, affine);

          tf::Transform T_o, T_g;
          tf::poseMsgToTF(goal.object.potential_models[0].pose.pose, T_o);
          tf::poseMsgToTF(goal.grasp.grasp_pose, T_g);
          tf::Transform T = T_g.inverse()*T_o;
          tf::poseTFToMsg(T, control_offset_.pose);

        }
      }

      if(use_cloud)
      {
        // Store point cloud

        sensor_msgs::PointCloud2 converted_cloud;
        sensor_msgs::convertPointCloudToPointCloud2 (goal.object.cluster, converted_cloud);

        pcl::fromROSMsg(converted_cloud, *cloud);
      }

      geometry_msgs::Pose &m = goal.grasp.grasp_pose;
      Eigen::Affine3f affine = Eigen::Translation3f(m.position.x,
                                                          m.position.y,
                                                          m.position.z) *
                                   Eigen::Quaternionf(m.orientation.w,
                                                      m.orientation.x,
                                                      m.orientation.y,
                                                      m.orientation.z);
      affine = affine.inverse();
      pcl::transformPointCloud(*cloud, *object_cloud_, affine);
    }
    catch(...){
      ROS_ERROR("%s: Error converting graspable object to reference frame id [%s]!",
                get_pose_name_.c_str(), goal.object.reference_frame_id.c_str());
    }
  }
  if (goal.gripper_pose.pose.orientation.x == 0 &&
      goal.gripper_pose.pose.orientation.y == 0 &&
      goal.gripper_pose.pose.orientation.z == 0 &&
      goal.gripper_pose.pose.orientation.w == 0 )
  {
    ROS_INFO("Empty pose passed in; using default");
    selected_pose_ = getDefaultPose();
    //ROS_DEBUG_STREAM("Default was \n" << selected_pose_);
    //transformGripperPose();
    //ROS_DEBUG_STREAM("Default after transform is\n" << selected_pose_);
    gripper_opening_ = 0.086;
    updateGripperAngle();
  }
  else
  {
    selected_pose_ = goal.gripper_pose;
    //transformGripperPose();
    gripper_opening_ = goal.gripper_opening;
    updateGripperAngle();
  }
  if (get_pose_server_.isPreemptRequested())
  {
    if(test_pose_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE ||
       test_pose_client_.getState() == actionlib::SimpleClientGoalState::PENDING )
    {
      test_pose_client_.cancelAllGoals();
    }
    get_pose_server_.setPreempted();
    setIdle();
    return;
  }

  if(use_haptics_)
  {
    setHapticDeviceTransform(fromWrist(selected_pose_));
  }
  else
  {
    // Run isosurface algorithm, which sets a new proxy pose.
    haptic_interface_.m_isosurface->reset();
    geometry_msgs::PoseStamped ps = fromWrist(selected_pose_);
    haptic_interface_.m_display->setClutchedPose(cml_tools::vectorMsgToCML(ps.pose.position),
                                                 cml_tools::quaternionMsgToCML(ps.pose.orientation));
    haptic_interface_.m_isosurface->update(haptic_interface_.m_display);
  }
  pose_state_ = UNTESTED;

  if(!use_haptics_) initMarkers();

  if (!getAndLoadCloudSnapshot()) return;
}

void GhostedGripperActionServer::getAndLoadCloudSnapshot( const std_msgs::StringConstPtr &cloud_name )
{
  if(cloud_name->data == "interactive_manipulation_snapshot")
  {
    getAndLoadCloudSnapshot();
  }
}


bool GhostedGripperActionServer::getAndLoadCloudSnapshot( )
{

  point_cloud_server::StoreCloudGoal cloud_goal;
  cloud_goal.action = cloud_goal.GET;
  cloud_goal.topic = "";
  cloud_goal.result_frame_id =    haptic_frame_id_;
  cloud_goal.name = "interactive_manipulation_snapshot";
  ROS_INFO("Sending request for cloud named [%s]", cloud_goal.name.c_str());
  cloud_server_client_.sendGoalAndWait(cloud_goal, ros::Duration(15.0), ros::Duration(5.0));

  if(cloud_server_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Got response from server!");
    ROS_INFO("Downsampling cloud...");
    ros::WallTime begin = ros::WallTime::now();
    // Create the filtering object
    sensor_msgs::PointCloud2::Ptr ptr (new sensor_msgs::PointCloud2 (cloud_server_client_.getResult()->cloud ));
    sensor_msgs::PointCloud2 filtered_cloud;

    //sensor_msgs::PointCloud2 temp =  cloud_server_client_.getResult()->cloud;
    //sensor_msgs::PointCloud2::Ptr ptr(&temp);
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
    sor.setInputCloud (ptr);
    sor.setLeafSize (voxel_size_, voxel_size_, voxel_size_);
    sor.filter (filtered_cloud);
    ROS_DEBUG_NAMED("cloud_handler", "Downsampling took %.3lf ms", (ros::WallTime::now() - begin).toSec()*1000.0);
    //msg_cloud_ = cloud_server_client_.getResult()->cloud;


    pcl::PointCloud<PointT>::Ptr snapshot_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(filtered_cloud, *snapshot_cloud);

    // TODO: get snapshot from server
    haptic_interface_.loadPointCloud(snapshot_cloud);
  }
  else
  {
    ROS_ERROR("Server did not succeed, status %s", cloud_server_client_.getState().toString().c_str());
    cancelCB();
    return false;
  }
  return true;
}

void GhostedGripperActionServer::setHapticDeviceTransform(const geometry_msgs::PoseStamped &ps)
{
  haptic_interface_.m_isosurface->reset();

  ROS_DEBUG_STREAM("Setting initial haptic device pose to:\n" << ps);
  cml::vector3d t;
  t.set(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
  cml::matrix33d r;
  r.identity();

  haptic_interface_.m_display->setClutchOffsets(t, r);
//  haptic_interface_.m_display->toolPosition();
  haptic_interface_.m_display->setProxyPosition(t + haptic_interface_.m_display->devicePosition());
}


//! Callback to allow this action to get preempted by backend.
void GhostedGripperActionServer::preemptCB()
{
  ROS_INFO("%s: Preempted", get_pose_name_.c_str());
  haptic_interface_.pauseHaptics();
  test_pose_client_.cancelAllGoals();
  get_pose_server_.setPreempted();
  setIdle();
}

//! Translate the control pose to the wrist.
geometry_msgs::PoseStamped GhostedGripperActionServer::toWrist(const geometry_msgs::PoseStamped &ps)
{
  geometry_msgs::PoseStamped out;
  out.header = ps.header;
  tf::Transform T, P;
  tf::poseMsgToTF(ps.pose, P);
  tf::poseMsgToTF(control_offset_.pose, T);
  tf::poseTFToMsg( P*T.inverse(), out.pose);
  //out.pose = object_manipulator::msg::applyShift(ps.pose, tf::Vector3(-0.15,0,0));
  return out;
}

//! Translate to the control pose.
geometry_msgs::PoseStamped GhostedGripperActionServer::fromWrist(const geometry_msgs::PoseStamped &ps)
{
  geometry_msgs::PoseStamped out;
  out.header = ps.header;
  tf::Transform T, P;
  tf::poseMsgToTF(ps.pose, P);
  tf::poseMsgToTF(control_offset_.pose, T);
  tf::poseTFToMsg( P*T, out.pose);
  //out.pose = object_manipulator::msg::applyShift(ps.pose, tf::Vector3(-0.15,0,0));
  return out;
}


/** \brief Update the pose of certain markers.
  */
void GhostedGripperActionServer::updatePoses()
{
  server_.setPose("selected_marker", selected_pose_.pose, selected_pose_.header);
  server_.setPose("proxy_marker", proxy_pose_.pose, proxy_pose_.header);
  server_.setPose("object_cloud", selected_pose_.pose, selected_pose_.header);
}


// **** 10 ***** 20 ****** 30 ****** 40 ****** 50 ****** 60 ****** 70 ****** 80 ****** 90 ***** 100 ***** 110 ***** 120


void GhostedGripperActionServer::initObjectMarker()
{
  if(object_cloud_->points.size())
  {
    server_.insert(makeCloudMarker( "object_cloud", selected_pose_, 0.004, object_manipulator::msg::createColorMsg(0.2, 0.8, 0.2,1.0) ));
  }
}

void GhostedGripperActionServer::initGripperControl()
{
  geometry_msgs::PoseStamped ps = fromWrist(selected_pose_);
  ps.header.stamp = ros::Time(0);
  server_.insert(make6DofMarker( "gripper_controls", ps, 0.2, false, false),
                 boost::bind( &GhostedGripperActionServer::updateGripper, this, _1));
  //menu_gripper_.apply(server_, "gripper_controls");
}

void GhostedGripperActionServer::initSelectedMarker()
{
  float r,g,b;
  switch(pose_state_)
  {
  case INVALID:
    r = 1.0; g = 0.2; b = 0.2;
    break;
  case VALID:
    r = 0.2; g = 0.6; b = 0.8;
    break;
  default:
    r = 0.5; g = 0.5; b = 0.5;
  }
  std_msgs::ColorRGBA color = object_manipulator::msg::createColorMsg(r,g,b,1.0);

  if(!haptic_interface_.isReady())
    color = object_manipulator::msg::createColorMsg(1.0, 0.6, 0.25, 1.0);

  server_.insert(makeGripperMarker( "selected_marker", selected_pose_, 1.0, gripper_angle_, false, color));
                 // boost::bind( &GhostedGripperActionServer::gripperClickCB, this, _1));
  menu_selected_marker_.apply(server_, "selected_marker");
}

void GhostedGripperActionServer::initProxyMarker()
{
  std_msgs::ColorRGBA color = object_manipulator::msg::createColorMsg(1.0, 0.6, 0.25, 1.0);
  server_.insert(makeGripperMarker( "proxy_marker", proxy_pose_, 0.98, gripper_angle_, false, color, !haptic_interface_.isReady()),
                  boost::bind( &GhostedGripperActionServer::proxyClickCB, this, _1));
  menu_proxy_marker_.apply(server_, "proxy_marker");
}


//! Callback for pose updates from the controls.
void GhostedGripperActionServer::updateGripper( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ros::Time now = ros::Time(0);

  static bool last_ready_state = false;

  switch ( feedback->event_type )
  {
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_DEBUG_STREAM( "Marker is being moved, stored pose is invalidated." );
      test_pose_client_.cancelAllGoals();
      //dragging_ = true;
      pose_state_ = UNTESTED;
      initSelectedMarker();
      break;
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_DEBUG_STREAM( "Marker was released, storing pose and checking." );
      //dragging_ = false;
      selected_pose_ = proxy_pose_;
      initMarkers();
      if(haptic_interface_.isReady())
      {
        testing_current_grasp_ = true;
        testPose(selected_pose_, gripper_opening_);
      }
      break;
  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_DEBUG_STREAM("POSE_UPDATE in frame " << feedback->header.frame_id << std::endl << feedback->pose);
      //if(selected_pose_.header.frame_id.compare(feedback->header.frame_id))  fix = true;

      selected_pose_.pose = feedback->pose;
      selected_pose_.header = feedback->header;
      selected_pose_ = toWrist(selected_pose_);

      // Run isosurface algorithm, which sets a new proxy pose.
      geometry_msgs::PoseStamped ps = fromWrist(selected_pose_);

      haptic_interface_.m_display->setClutchedPose(cml_tools::vectorMsgToCML(ps.pose.position),
                                                   cml_tools::quaternionMsgToCML(ps.pose.orientation));
      haptic_interface_.m_isosurface->update(haptic_interface_.m_display);
      // Subscriber automatically gets the proxy pose.

      if(haptic_interface_.isReady() && !last_ready_state)  initSelectedMarker();
      last_ready_state = haptic_interface_.isReady();
      updatePoses();
      break;
  }
}


//! Callback that receives the result of a TestGripperPose action.
void GhostedGripperActionServer::testGripperResultCallback(const actionlib::SimpleClientGoalState& state,
                               const pr2_im_msgs::TestGripperPoseResultConstPtr &result)
{
  if (result->valid.empty())
  {
    ROS_ERROR("Test gripper pose returned with empty result list");
    return;
  }
  if(state.state_ == state.SUCCEEDED)
  {
    ROS_DEBUG("Test pose action returned with result %d", (int)result->valid[0]);
    PoseState state = INVALID;
    if (result->valid[0]) state = VALID;
    pose_state_ = state;
    initSelectedMarker();
  }
  else
  {
    ROS_WARN("Test pose action did not succeed; state = %d", (int)state.state_);
  }
}


//! Return with the gripper pose if the pose is valid, otherwise do nothing
void GhostedGripperActionServer::acceptCB()
{
  if( pose_state_ == VALID )
  {
    if(test_pose_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE ||
       test_pose_client_.getState() == actionlib::SimpleClientGoalState::PENDING )
    {
      test_pose_client_.cancelGoal();
    }
    haptic_interface_.pauseHaptics();
    setIdle();
    pr2_im_msgs::GetGripperPoseResult result;
    result.gripper_pose = selected_pose_;
    result.gripper_opening = gripper_opening_;
    get_pose_server_.setSucceeded(result);
  }
}

//! Gets rid of tested gripper pose
void GhostedGripperActionServer::clearAllMarkers()
{
  server_.erase("selected_marker");
  server_.erase("proxy_marker");
  server_.erase("gripper_controls");
  server_.erase("object_cloud");
}

//! Cancel this action call
void GhostedGripperActionServer::cancelCB()
{
  if(test_pose_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE ||
     test_pose_client_.getState() == actionlib::SimpleClientGoalState::PENDING )
  {
    test_pose_client_.cancelGoal();
  }
  haptic_interface_.pauseHaptics();
  get_pose_server_.setAborted();
  setIdle();
}

  //! Called when the proxy is clicked
  void GhostedGripperActionServer::proxyClickCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    if(!haptic_interface_.isReady()) return;

     ros::Time now = ros::Time(0);

    switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_DEBUG( "Clicked on proxy, setting selected pose to proxy pose.");

      selected_pose_ = proxy_pose_;
      pose_state_ = UNTESTED;
      initMarkers();
      testing_current_grasp_ = true;
      testPose(selected_pose_, gripper_opening_);
      break;
    }
  }

//  //! Called when the gripper is clicked; each call cycles through gripper opening values.
//  void gripperClickCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
//  {
//    if(interface_number_ == 2) return;
//
//    ros::Time now = ros::Time(0);
//
//    switch ( feedback->event_type )
//    {
//    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
//        //ROS_DEBUG_STREAM( "Marker is being moved, stored pose is invalidated." );
//        float max_gripper_angle = 0.541;
//        gripper_angle_ -= 0.12;
//        if(gripper_angle_ < 0.04)
//          gripper_angle_ = max_gripper_angle;
//        updateGripperOpening();
//        initSelectedMarker();
//        ROS_DEBUG( "Gripper opening = %.2f, angle = %.2f", gripper_opening_, gripper_angle_);
//        break;
//    }
//  }

void GhostedGripperActionServer::testPose(geometry_msgs::PoseStamped pose, float opening)
{
  pr2_im_msgs::TestGripperPoseGoal goal;
  goal.gripper_poses.push_back(pose);
  goal.gripper_openings.push_back(opening);
  test_pose_client_.sendGoal( goal, boost::bind(&GhostedGripperActionServer::testGripperResultCallback, this, _1, _2));
}



//! Initialize the menus for all markers.
void GhostedGripperActionServer::initMenus()
{

  accept_handle_ = menu_selected_marker_.insert("Accept", boost::bind( &GhostedGripperActionServer::acceptCB, this ) );
  menu_selected_marker_.insert("- - - - - -", boost::bind( &GhostedGripperActionServer::acceptCB, this ) );
  if(use_haptics_) menu_selected_marker_.insert("Clear", boost::bind( &GhostedGripperActionServer::clearAllMarkers, this ) );
  cancel_handle_ = menu_selected_marker_.insert("Cancel", boost::bind( &GhostedGripperActionServer::cancelCB, this ) );
}

bool GhostedGripperActionServer::getModelMesh( int model_id, arm_navigation_msgs::Shape& mesh )
{
  household_objects_database_msgs::GetModelMesh mesh_srv;

  mesh_srv.request.model_id = model_id;
  if ( !get_model_mesh_client_.call(mesh_srv) )
  {
    ROS_ERROR("Failed to call get model mesh service");
    return false;
  }

  if (mesh_srv.response.return_code.code != household_objects_database_msgs::DatabaseReturnCode::SUCCESS)
  {
    ROS_ERROR("Model mesh service reports an error (code %d)", mesh_srv.response.return_code.code);
    return false;
  }

  mesh = mesh_srv.response.mesh;
  return true;
}



//! Create an interactive marker from a point cloud.
visualization_msgs::InteractiveMarker GhostedGripperActionServer::makeCloudMarker( const char *name, const geometry_msgs::PoseStamped &stamped,
                                                       float point_size, std_msgs::ColorRGBA color)
{
  InteractiveMarker int_marker;
  int_marker.name = name;
  int_marker.pose = stamped.pose;
  int_marker.header = stamped.header;

  Marker marker;
  marker.color = color;
  marker.frame_locked = false;

  if(object_cloud_->points.size())
  {
    //int_marker.header = object_cloud_->header;
    marker.scale.x = point_size;
    marker.scale.y = point_size;
    marker.scale.z = point_size;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;

    int num_points = object_cloud_->points.size();
    marker.points.resize( num_points );
//      marker.colors.resize( num_points );

    //ROS_INFO_STREAM( "Adding point cluster. #points=" << object_.cluster.points.size() );

    for ( int i=0; i<num_points; i++)
    {
      marker.points[i].x = object_cloud_->points[i].x;
      marker.points[i].y = object_cloud_->points[i].y;
      marker.points[i].z = object_cloud_->points[i].z;
//        marker.colors[i].r = object_cloud_->points[i].r/255.;
//        marker.colors[i].g = object_cloud_->points[i].g/255.;
//        marker.colors[i].b = object_cloud_->points[i].b/255.;
//        marker.colors[i].a = 1.0;
    }
  }
  else
  {

  }

  InteractiveMarkerControl control;
  control.always_visible = true;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;

  control.markers.push_back( marker );

  int_marker.controls.push_back( control );

  return int_marker;
}
