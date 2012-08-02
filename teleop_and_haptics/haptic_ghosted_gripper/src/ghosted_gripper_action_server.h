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

#ifndef _GHOSTED_GRIPPER_ACTION_SERVER_H_
#define _GHOSTED_GRIPPER_ACTION_SERVER_H_

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

#include <pr2_object_manipulation_msgs/TestGripperPoseAction.h>
#include <pr2_object_manipulation_msgs/GetGripperPoseAction.h>

#include <sensor_msgs/point_cloud_conversion.h>

#include "eigen_conversions/eigen_msg.h"
#include "eigen3/Eigen/Geometry"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl_ros/transforms.h"

#include <household_objects_database_msgs/GetModelDescription.h>
#include <household_objects_database_msgs/GetModelMesh.h>

#include "marker_helpers.h"

#include "MyHapticsWidget.h"

//using namespace object_manipulator;
//using namespace visualization_msgs;
//using namespace interactive_markers;
//using namespace pr2_object_manipulation_msgs;



////////////////////////////////////////////////////////////////////////////////////


/** \brief An action for getting a gripper pose using interactive markers.
  */
class GhostedGripperActionServer
{
protected:

  // ****************** class members *********************

  geometry_msgs::PoseStamped selected_pose_, proxy_pose_;
  geometry_msgs::PoseStamped control_offset_;
  float gripper_opening_;
  float gripper_angle_;
  pcl::PointCloud<PointT>::Ptr object_cloud_;
  bool active_;
  bool object_model_;
  bool use_haptics_;

  bool testing_current_grasp_;

  PoseState pose_state_;

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_seed_, sub_selected_pose_, sub_proxy_pose_, sub_refresh_flag_;
  ros::ServiceClient get_model_mesh_client_;
  ros::Timer spin_timer_;
  interactive_markers::InteractiveMarkerServer server_;
  double voxel_size_;
  std::string haptic_frame_id_;

  ros::Publisher pub_cloud_;

  interactive_markers::MenuHandler menu_selected_marker_;
  interactive_markers::MenuHandler menu_proxy_marker_;
  interactive_markers::MenuHandler::EntryHandle accept_handle_;
  interactive_markers::MenuHandler::EntryHandle cancel_handle_;

  tf::TransformListener tfl_;

  object_manipulator::MechanismInterface mechanism_;

  actionlib::SimpleActionClient<pr2_object_manipulation_msgs::TestGripperPoseAction> test_pose_client_;
  actionlib::SimpleActionClient<point_cloud_server::StoreCloudAction> cloud_server_client_;

  std::string get_pose_name_;
  actionlib::SimpleActionServer<pr2_object_manipulation_msgs::GetGripperPoseAction> get_pose_server_;

  HapticGhostedGripper haptic_interface_;

public:

  GhostedGripperActionServer(bool use_haptics);
  ~GhostedGripperActionServer() {};

  void updateGripperOpening() {  gripper_opening_ = gripper_angle_ * 0.1714;  }

  void updateGripperAngle()   {  gripper_angle_ = gripper_opening_ * 5.834;   }

  void setSeed(const geometry_msgs::PoseStampedConstPtr &seed);

  bool transformPoseToCommonFrame(const geometry_msgs::PoseStamped &ps,
                                                              geometry_msgs::PoseStamped &result);

  void setSelectedPose(const geometry_msgs::PoseStampedConstPtr &seed);

  void setProxyPose(const geometry_msgs::PoseStampedConstPtr &seed);

  //! Remove the markers.
  void setIdle();

  //! Callback to accept a new action goal.
  void goalCB();

  //! Callback to allow this action to get preempted by backend.
  void preemptCB();

  //! Translate the control pose to the wrist.
  geometry_msgs::PoseStamped toWrist(const geometry_msgs::PoseStamped &ps);


  //! Translate to the control pose.
  geometry_msgs::PoseStamped fromWrist(const geometry_msgs::PoseStamped &ps);

  //! set the transform pose for the haptic tool
  void setHapticDeviceTransform(const geometry_msgs::PoseStamped &ps);

  //! Retrieves cloud snapshot from server and loads into haptic scene.
  bool getAndLoadCloudSnapshot();
  void getAndLoadCloudSnapshot( const std_msgs::StringConstPtr &cloud_name );

  //! Transmit gripper poses
  void updatePoses();

  //geometry_msgs::PoseStamped getDefaultPose(std::string arm_name)


// **** 10 ***** 20 ****** 30 ****** 40 ****** 50 ****** 60 ****** 70 ****** 80 ****** 90 ***** 100 ***** 110 ***** 120

  /** \brief Re-initializes all markers.
    */
  void initMarkers()
  {
    if(!use_haptics_)
    {
      initGripperControl();
      //initProxyMarker();
    }
    initSelectedMarker();
    initObjectMarker();
  }

  void initGripperControl();
  void initObjectMarker();
  void initSelectedMarker();
  void initProxyMarker();

  geometry_msgs::PoseStamped getDefaultPose();

protected:


  //! ROS spin update callback
  void spinOnce()
  {
    server_.applyChanges();
  }


  //! Callback that receives the result of a TestGripperPose action.
  void testGripperResultCallback(const actionlib::SimpleClientGoalState& state,
                                 const pr2_object_manipulation_msgs::TestGripperPoseResultConstPtr &result);


  //! Return with the gripper pose if the pose is valid, otherwise do nothing
  void acceptCB();


  //! Cancel this action call
  void cancelCB();

  //! Clears the tested pose
  void clearAllMarkers();


//  //! Called when the gripper is clicked; each call cycles through gripper opening values.
//  void gripperClickCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )

  void proxyClickCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void updateGripper( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void testPose(geometry_msgs::PoseStamped pose, float opening);

  //! Initialize the menus for all markers.
  void initMenus();

  bool getModelMesh( int model_id, arm_navigation_msgs::Shape& mesh );

  //! Create an interactive marker from a point cloud.
  visualization_msgs::InteractiveMarker makeCloudMarker( const char *name, const geometry_msgs::PoseStamped &stamped,
                                                         float point_size, std_msgs::ColorRGBA color);


};


#endif

