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

#include <ros/ros.h>
#include <boost/bind.hpp>
//#include "MyHapticsWidget.h"
#include "ghosted_gripper_action_server.h"
#include "MyHapticsThread.h"


int main (int argc, char* argv[])
{
  printf("Starting...\n");
  ros::init(argc, argv, "haptics_ghosted_gripper_node");

  // create and start a thread for haptics
  ROS_INFO("Creating haptics thread...");
  MyHapticsThread *hthread = MyHapticsThread::instance();
  hthread->setHapticRate(2000);
  hthread->start(QThread::TimeCriticalPriority);

//  // run the application
//  MyMainWindow *window = MyMainWindow::instance();
//  window->show();
//  int result = application.exec();


//  HapticGhostedGripper* haptics = new HapticGhostedGripper();
////  ros::MultiThreadedSpinner spinner(3); // Use 4 threads
////  spinner.spin();
//  ros::spin();

  ROS_INFO("Creating GhostedGripperActionServer...");
  GhostedGripperActionServer ggas;
  ROS_INFO("...spinning!");
  ros::spin();

  // stop the haptic servo thread and destroy it
  hthread->quit();
  hthread->resume();
  hthread->wait();
  delete hthread;

  return (0);
}
