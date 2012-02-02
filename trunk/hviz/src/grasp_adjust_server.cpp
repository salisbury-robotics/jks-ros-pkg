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
///\brief Finds optimal grasps near a provided grasp pose.


#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <boost/bind.hpp>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl_tf/transforms.h>



#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include "pr2_gripper_grasp_adjust/EstimateConfig.h"
#include "pr2_gripper_grasp_adjust/DebugConfig.h"
#include "pr2_gripper_grasp_adjust/GripperGraspAdjust.h"


#include <iostream>
#include <fstream>
#include <queue>

#include "gripper_model.h"
#include "helpers.h"
#include "grasp_adjust.h"

/*
#define PROF_ENABLED
#include <profiling/profiling.h>
PROF_DECLARE(TOTAL_TIMER)
PROF_DECLARE(TRANSFORM_INPUT_CLOUD)
PROF_DECLARE(DO_DESCENT)
PROF_DECLARE(PROF_1)
*/

typedef pcl::PointXYZRGBNormal PointT;

const int GLOBAL_SEARCH =  pr2_gripper_grasp_adjust::Estimate_global_search;
const int LOCAL_SEARCH =  pr2_gripper_grasp_adjust::Estimate_local_search;
const int SINGLE_POSE =  pr2_gripper_grasp_adjust::Estimate_single_pose;


class GraspAdjustServer{

    pcl::PointCloud<PointT> last_cloud_;

    ros::NodeHandle nh_, nh_pvt_;
    ros::Subscriber sub_cloud_;
    ros::Publisher pub_cloud_, pub_marker_, pub_marker_array_;
    ros::Publisher pub_cloud_roi_, pub_cloud_debug_2;

    ros::ServiceServer adjust_srv_;
    ros::Timer update_timer_;

    std::string input_topic_;
    std::string output_topic_;

    tf::TransformListener listener_;
    tf::TransformBroadcaster broadcaster_;

    ros::Time now_;

    bool got_first_cloud_;

    GraspAdjust grasp_adjust_;

    // ***** Dynamic reconfigure stuff *****
    typedef pr2_gripper_grasp_adjust::EstimateConfig Config;
    dynamic_reconfigure::Server<Config>                dyn_srv;
    dynamic_reconfigure::Server<Config>::CallbackType  dyn_cb;

    typedef pr2_gripper_grasp_adjust::DebugConfig Debug;
    dynamic_reconfigure::Server<Debug>                debug_srv;
    dynamic_reconfigure::Server<Debug>::CallbackType  debug_cb;


    // ***** Helpers for training ******
    bool training_;
    ofstream train_file_;

public:
    /*!
    * \brief Default contructor.
    *
    * This description is displayed lower in the doxygen as an extended description along with
    * the above brief description.
    */
    GraspAdjustServer(char* argv[]):
            nh_("/"),
            nh_pvt_("~"),
            input_topic_("/cloud_in"),
            output_topic_("/cloud_out"),
            dyn_srv(ros::NodeHandle("~config")),
            debug_srv(ros::NodeHandle("~debug")),
            training_(0)
    {
        got_first_cloud_ = false;
        now_ = ros::Time::now() - ros::Duration(1.0);

        // Clouds in and out
        sub_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>(input_topic_, 1, boost::bind(&GraspAdjustServer::cloud_callback, this, _1));
        pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
        pub_cloud_roi_ = nh_.advertise<sensor_msgs::PointCloud2>("/roi_snapshot", 1);

        // Marker topics
        pub_marker_ = nh_.advertise<visualization_msgs::Marker>("/markers", 1);
        pub_marker_array_ = nh_.advertise<visualization_msgs::MarkerArray>("/markers_array", 1);

        grasp_adjust_.pub_cloud_ = &pub_cloud_;
        grasp_adjust_.pub_cloud_roi_ = &pub_cloud_roi_;
        grasp_adjust_.pub_marker_ = &pub_marker_;
        grasp_adjust_.pub_marker_array_ = &pub_marker_array_;
        grasp_adjust_.listener_ = &listener_;
        grasp_adjust_.broadcaster_ = &broadcaster_;

        // Create callback for auto-running of algorithm
        update_timer_ = nh_.createTimer(ros::Duration(0.5), boost::bind(&GraspAdjustServer::timerCallback, this));

        bool load_debug_server = false;
        nh_pvt_.param("load_debug_server", load_debug_server, load_debug_server);

        // This server is for the user to adjust the algorithm weights
        dyn_cb = boost::bind( &GraspAdjustServer::dynamicCallback, this, _1, _2 );
        dyn_srv.setCallback(dyn_cb);

        // This server is used for debugging the geometry and algorithm code.
        if(load_debug_server){
            ROS_INFO("Loading the debug dynamic reconfigure server");
            debug_cb = boost::bind( &GraspAdjustServer::debugCallback, this, _1, _2 );
            debug_srv.setCallback(debug_cb);
        }

        adjust_srv_ = nh_.advertiseService("pr2_gripper_grasp_adjust", &GraspAdjustServer::graspAdjustCallback, this);

        // if training, open the output file
        if(training_){
            ROS_INFO("in training mode, opening training_output.txt");
            train_file_.open("/u/hsiao/gripper_stereo_data/training_data/training_output.txt");
        }

        ROS_INFO("Finished constructor!");
    }


private:

    /*!
    * \brief The timer callback allows you to run the adjustment algorithm continuously.
    *
    * This description is displayed lower in the doxygen as an extended description along with
    * the above brief description.
    */
    void timerCallback()
    {
        now_= last_cloud_.header.stamp;
        geometry_msgs::PoseStamped pose_msg;
        tf::Pose pose;

        if(got_first_cloud_ && grasp_adjust_.debug_.run_continuous)
        {
            if(grasp_adjust_.debug_.custom_pose)
            {
                if(grasp_adjust_.debug_.custom_frame.empty())
                    pose_msg.header.frame_id = "torso_lift_link";
                else
                    pose_msg.header.frame_id = grasp_adjust_.debug_.custom_frame;
                pose_msg.header.stamp = now_;

                float x = grasp_adjust_.debug_.custom_x;
                float y = grasp_adjust_.debug_.custom_y;
                float z = grasp_adjust_.debug_.custom_z;
                float roll = grasp_adjust_.debug_.custom_roll     * M_PI/180.0;
                float pitch = grasp_adjust_.debug_.custom_pitch   * M_PI/180.0;
                float yaw = grasp_adjust_.debug_.custom_yaw       * M_PI/180.0;

                tf::Transform T_rot;
                tf::Transform T_offset;
                tf::Transform T_trans = tf::Transform(tf::Quaternion(0,0,0,1),
                                                      tf::Vector3(x, y, z));
                T_rot = tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw),
                                      tf::Vector3( 0, 0, 0) );
                T_offset = T_trans * grasp_adjust_.DEFAULT_GRIPPER.tool_frame_ * T_rot * grasp_adjust_.DEFAULT_GRIPPER.tool_frame_.inverse();
                pose = T_offset;
            }
            else // Set a pose 6cm in front of current gripper pose
            {
                pose_msg.header.frame_id = "r_wrist_roll_link";
                pose_msg.header.stamp = now_;;
                pose = tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(0.06, 0, 0));
            }

            tf::poseTFToMsg(pose, pose_msg.pose);
            vector<geometry_msgs::PoseStamped> adjusted_poses;

            int value = grasp_adjust_.findGrasps(pose_msg, &adjusted_poses, grasp_adjust_.config_.search_mode);
            ROS_INFO("pr2_gripper_grasp_adjust::grasp_adjust returned %d", value);
        }
    }

    /*!
    * \brief Service callback for the grasp adjust service.
    *
    * This description is displayed lower in the doxygen as an extended description along with
    * the above brief description.
    * \param req    The service request
    * \param res    The response
    */
    bool graspAdjustCallback(   pr2_gripper_grasp_adjust::GripperGraspAdjust::Request  &req,
                                pr2_gripper_grasp_adjust::GripperGraspAdjust::Response &res )
    {

        int mode = req.do_global_search?(GLOBAL_SEARCH):(LOCAL_SEARCH);
        res.result = grasp_adjust_.findGrasps(req.grasp_pose, &(res.adjusted_grasp_pose), mode);

        ROS_INFO("pr2_gripper_grasp_adjust::grasp_adjust returned %d", res.result);

        return true;
    }


    /*!
    * \brief Callback for receiving a point cloud.
    *
    * This description is displayed lower in the doxygen as an extended description along with
    * the above brief description.
    */
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &input)
    {
        ROS_DEBUG("Got a cloud with %d points!", input->width*input->height);

        pcl::fromROSMsg(*input, last_cloud_);
        grasp_adjust_.setInputCloud(last_cloud_);

        if(!got_first_cloud_){
            got_first_cloud_ = true;
            ROS_INFO("Got first cloud!");
        }
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
                new_config = grasp_adjust_.config_;
                break;

              default:  // Error condition
                ROS_INFO("Dynamic reconfigure did something, variable %d.", id);

          }
          grasp_adjust_.config_ = new_config;
          new_config.status = status;
      }


    /*!
    * \brief Callback for setting debug gripper poses through dynamic reconfigure.
    *
    * This description is displayed lower in the doxygen as an extended description along with
    * the above brief description.
    */
    void debugCallback(Debug &new_config, uint32_t id)
      {
          // This info will get printed back to reconfigure gui
          char status[256] = "\0";

          switch(id){
              case(-1): // Init
                // If you are tempted to put anything here, it should probably go in the constructor
                break;

              case(0): // Connect
                printf("Reconfigure GUI connected to me!\n");
                new_config = grasp_adjust_.debug_;
                break;

              default:  // Error condition
                ROS_INFO("Dynamic reconfigure did something, variable %d.", id);

          }
          grasp_adjust_.debug_ = new_config;
          new_config.status = status;
      }
};



/* ---[ */
int main (int argc, char* argv[])
{
  ros::init(argc, argv, "filter");

  GraspAdjustServer server(argv);

  ros::spin();

  return (0);
}
/* ]--- */
