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
///\brief Node for simple haptic interaction

#include "chai3d.h"
#include "CThreadWithArgument.h"

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


#include <iostream>
#include <fstream>
#include <queue>

#define PROF_ENABLED
#include <profiling/profiling.h>
PROF_DECLARE(TOTAL_TIMER)
PROF_DECLARE(FUNC_1)



//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a haptic device handler

//cGeneric3dofPointer* tool;


// label to show estimate of haptic update rate
double rateEstimate = 0;
double graphicRateEstimate = 0;
bool simulationRunning = false;
bool simulationFinished = false;

void hapticsDispatch(void *instance);
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

namespace cml_tools
{

  tf::Vector3 cmlVectorToTF(const cml::vector3d &v)
{
  tf::Vector3 vtf;
  vtf.setX(v[0]);
  vtf.setY(v[1]);
  vtf.setZ(v[2]);
  return vtf;
}

btMatrix3x3 cmlMatrixToTF(const cml::matrix33d &rot)
{
  return btMatrix3x3( rot.data()[0], rot.data()[1], rot.data()[2],
                      rot.data()[3], rot.data()[4], rot.data()[5],
                      rot.data()[6], rot.data()[7], rot.data()[8] );
}

btQuaternion cmlMatrixToTFQuaternion(const cml::matrix33d &m)
{
  tf::Quaternion q;
  cmlMatrixToTF(m).getRotation(q);
  return q;
}

} // namespace chai_tools

class HapticNode{

   ros::NodeHandle nh_, pnh_;

  // Publishers for markers (e.g. haptic tool)
  ros::Publisher pub_marker_, pub_marker_array_;

  // Publisher for string status data
  ros::Publisher pub_status_;

  // A timer callback
  ros::Timer update_timer_;

  tf::TransformListener tfl_;
  tf::TransformBroadcaster tfb_;

  ros::Time now_;


  std::string tool_frame_;

  HapticDisplay *tool_;
  cHapticDeviceHandler* handler_;
  cHapticDeviceInfo device_info_;

public:

  //! The constructor
  HapticNode(char* argv[]):
      nh_("/"), pnh_("~"),
      tool_frame_("/base_link")
  {
    now_ = ros::Time::now() - ros::Duration(1.0);

    // Marker topics
    pub_marker_ = pnh_.advertise<visualization_msgs::Marker>("/markers", 100);
    pub_marker_array_ = pnh_.advertise<visualization_msgs::MarkerArray>("/markers_array", 1);

    // String status topic
    pub_status_ = pnh_.advertise<std_msgs::String>("status", 10);

    // Create timer callback for auto-running of algorithm
    update_timer_ = nh_.createTimer(ros::Duration(0.03333), boost::bind(&HapticNode::displayCallback, this));

    // Set up the chai world and device
    initializeHaptics();

    ROS_INFO("Finished constructor!");
  }

  ~HapticNode()
  {
    ROS_INFO("Closing down...");

  }

  void initializeHaptics()
  {
    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    ROS_INFO("Creating connection to device...");
    // create a haptic device handler
    handler_ = new cHapticDeviceHandler();

    // get access to the first available haptic device
    cGenericHapticDevice* hapticDevice;

    handler_->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    if (hapticDevice)
    {
        device_info_ = hapticDevice->getSpecifications();
    }

    tool_ = new HapticDisplay(hapticDevice, 0);
    tool_->setToolRadius(0.08);
    ROS_INFO("Setting device workspace to %f", tool_->deviceWorkspace());
    tool_->setWorkspaceRadius(tool_->deviceWorkspace());

    ROS_INFO("Setting stiffness to %f", device_info_.m_maxForceStiffness);
    tool_->setStiffness(device_info_.m_maxForceStiffness * 0.05);

    cml::matrix33d permutation;
    permutation.identity();
    tool_->setPermutation(permutation);

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

  //! The display callback sends graphical output to rviz.
  void displayCallback()
  {
    if(!tool_) return;

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
        //ROS_INFO_STREAM(status_string);
        msg.data = status_string;
        pub_status_.publish(msg);
    }

    // Transmit the visualizations of the tool/proxy
    float proxy_radius = tool_->toolRadius();
    cml::vector3d proxy_pos =  tool_->proxyPosition();
    cml::matrix33d proxy_rot = tool_->proxyOrientation();
    cml::vector3d HIP_pos = tool_->toolPosition();
    cml::matrix33d HIP_rot = tool_->toolOrientation();

    if(device_info_.m_sensedRotation)
    {
      object_manipulator::shapes::Arrow arrow;
      arrow.dims = tf::Vector3(2*proxy_radius, proxy_radius/2, proxy_radius/2);
      arrow.frame.setRotation(cml_tools::cmlMatrixToTFQuaternion(proxy_rot));
      arrow.frame.setOrigin(cml_tools::cmlVectorToTF(proxy_pos));
      arrow.header.stamp = time_now;
      arrow.header.frame_id = tool_frame_.c_str();
      object_manipulator::drawArrow(pub_marker_, arrow, "proxy", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 0.3, 0.7, 1.0));

      arrow.dims = tf::Vector3(2*proxy_radius, proxy_radius/2, proxy_radius/2);
      arrow.frame.setRotation(cml_tools::cmlMatrixToTFQuaternion(HIP_rot));
      arrow.frame.setOrigin(cml_tools::cmlVectorToTF(HIP_pos));
      arrow.header.stamp = time_now;
      arrow.header.frame_id = tool_frame_.c_str();
      object_manipulator::drawArrow(pub_marker_, arrow, "HIP", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 0.4, 0.2, 0.6));
    }
    else
    {
//      object_manipulator::shapes::Sphere sphere;
//      sphere.dims = tf::Vector3(2*proxy_radius, 2*proxy_radius, 2*proxy_radius);
//      sphere.frame = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(pos.x, pos.y, pos.z));
//      sphere.header.frame_id = tool_frame_.c_str();
//      sphere.header.stamp = time_now;
//      object_manipulator::drawSphere(pub_marker_, sphere, "proxy", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 0.3, 0.7, 1.0));
//
//      //object_manipulator::shapes::Sphere sphere;
//      sphere.dims = tf::Vector3(1.9*proxy_radius, 1.9*proxy_radius, 1.9*proxy_radius);
//      sphere.frame = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(HIP.x, HIP.y, HIP.z));
//      sphere.header.frame_id = tool_frame_.c_str();
//      sphere.header.stamp = time_now;
//      object_manipulator::drawSphere(pub_marker_, sphere, "HIP", 0, ros::Duration(), object_manipulator::msg::createColorMsg(1.0, 0.4, 0.2, 0.6));
    }

  }

  //! The haptic update loop.
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

        cml::matrix33d I3;
        I3.identity();
        tool_->update();
        tool_->setProxyPosition(cml::vector3d(0,0,0));
        tool_->setProxyOrientation(I3);
        tool_->applyForces();
//        tool_->setProxyPosition(tool_->toolPosition());
//        tool_->setProxyOrientation(tool_->toolOrientation());

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

};  // End of class definition


void hapticsDispatch(void *instance)
{
    HapticNode *widget = reinterpret_cast<HapticNode *>(instance);
    if (widget) widget->updateHaptics();
}

/* ---[ */
int main (int argc, char* argv[])
{
  ros::init(argc, argv, "haptics_ghosted_gripper_node");
  HapticNode haptics(argv);
  ros::MultiThreadedSpinner spinner(3); // Use 4 threads
  spinner.spin();

  return (0);
}
/* ]--- */
