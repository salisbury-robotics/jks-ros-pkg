#include <ros/ros.h>
#include "eigen_conversions/eigen_msg.h"

#include <object_manipulator/tools/mechanism_interface.h>
#include <object_manipulator/tools/msg_helpers.h>

#include "eigen3/Eigen/Geometry"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl_ros/transforms.h"

#include <tf/transform_listener.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <point_cloud_server/StoreCloudAction.h>
#include <pr2_im_msgs/TestGripperPoseAction.h>
#include <pr2_im_msgs/GetGripperPoseAction.h>

#include <household_objects_database_msgs/GetModelDescription.h>
#include <household_objects_database_msgs/GetModelMesh.h>


typedef pcl::PointXYZ PointT;

class HapticGripper{

protected:

  //! States for a pose
  enum PoseState {UNTESTED, VALID, INVALID};

  //! Gripper state
  geometry_msgs::PoseStamped gripper_pose_;
  float gripper_opening_;
  float gripper_angle_;

  bool active_;
  PoseState pose_state_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::ServiceClient get_model_mesh_client_;
  //ros::Timer spin_timer_;
  //ros::Timer slow_sync_timer_;
  InteractiveMarkerServer server_;

  ros::Publisher pub_cloud_;

  MenuHandler menu_gripper_;
  MenuHandler::EntryHandle accept_handle_;
  MenuHandler::EntryHandle cancel_handle_;

  tf::TransformListener tfl_;

  object_manipulator::MechanismInterface mechanism_;

  actionlib::SimpleActionClient<pr2_im_msgs::TestGripperPoseAction> test_pose_client_;
  actionlib::SimpleActionClient<point_cloud_server::StoreCloudAction> cloud_server_client_;

  std::string get_pose_name_;
  actionlib::SimpleActionServer<pr2_im_msgs::GetGripperPoseAction> get_pose_server_;

public:

  HapticGripper() :
      object_cloud_(new pcl::PointCloud<PointT>()),
      active_(false),
      object_model_(false),
      nh_("/"),
      pnh_("~"),
      server_("haptic_gripper", "server 1", false),
      tfl_(nh_),
      test_pose_client_("test_gripper_pose", true),
      cloud_server_client_("point_cloud_server_action", true),
      get_pose_name_(ros::this_node::getName()),
      get_pose_server_(nh_, get_pose_name_, false)
  {
    ROS_INFO( "haptic gripper IM server is running." );

    ros::Duration(1.0).sleep();
    pose_state_ = UNTESTED;
    //gripper_pose_ = getDefaultPose("right_arm");
    gripper_angle_ = 0.541;
    initMenus();
    //initMarkers();
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "/base_link";
    server_.insert(makeButtonBox( "test_box", ps, 0.01, false, false));

    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_debug", 1);

    get_model_mesh_client_ = nh_.serviceClient<household_objects_database_msgs::GetModelMesh>("objects_database_node/get_model_mesh", false);

    spin_timer_ =  nh_.createTimer(ros::Duration(0.05), boost::bind( &HapticGripper::spinOnce, this ) );

    planner_index_ = 0;

    //register the goal and feeback callbacks
    get_pose_server_.registerGoalCallback(    boost::bind(&HapticGripper::goalCB, this));
    get_pose_server_.registerPreemptCallback( boost::bind(&HapticGripper::preemptCB, this));

    get_pose_server_.start();
  }

  ~HapticGripper()
  {
  }

  bool transformGripperPose(const std::string frame_id = "/base_link")
  {
  /*  try{
      tfl_.waitForTransform(frame_id, gripper_pose_.header.frame_id, gripper_pose_.header.stamp, ros::Duration(3.0));
      tfl_.transformPose(frame_id, gripper_pose_, gripper_pose_);
      return true;
    }
    catch(...){
      ROS_ERROR("TF can't transform!");
      return false;
    }
*/
    return true;
  }

  void updateGripperOpening()
  {
    gripper_opening_ = gripper_angle_ * 0.1714;
  }

  void updateGripperAngle()
  {
    gripper_angle_ = gripper_opening_ * 5.834;
  }

  void setSeed(const geometry_msgs::PoseStampedConstPtr &seed)
  {
    if(!active_) return;
    ROS_DEBUG("Setting seed.");
    geometry_msgs::PoseStamped ps = *seed;
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
    //ps.header = seed->header;
    //ROS_INFO_STREAM("Processed seed before wrist offset was \n" << ps);

    gripper_pose_ = toWrist(ps);
    //ROS_INFO_STREAM("Processed seed was \n" << gripper_pose_);
    //transformGripperPose();
    //ROS_DEBUG_STREAM("But after transforming it is \n" << gripper_pose_);
    //ROS_INFO_STREAM("ansforming it is \n" << gripper_pose_);
    //transformGripperPose();

    pose_state_ = UNTESTED;

    if(always_call_planner_)
    {
      graspPlanCB();
    }
    else
    {
      initMarkers();
      testPose();
    }
  }

  //! Remove the markers.
  void setIdle(){
    server_.erase("ghosted_gripper");
    server_.erase("gripper_controls");
    server_.erase("object_cloud");
    server_.erase("grasp_toggle");
    active_ = false;
  }


  //! Callback to accept a new action goal.
  void goalCB()
  {
    active_ = true;
    object_model_ = false;
    planner_index_ = 0;
    ROS_INFO("Ghosted gripper called");
    pr2_marker_control::GetGripperPoseGoal goal = *get_pose_server_.acceptNewGoal();
    object_cloud_.reset( new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
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
      gripper_pose_ = getDefaultPose(goal.arm_name);
      //ROS_DEBUG_STREAM("Default was \n" << gripper_pose_);
      //transformGripperPose();
      //ROS_DEBUG_STREAM("Default after transform is\n" << gripper_pose_);
      gripper_opening_ = 0.08;
    }
    else
    {
      gripper_pose_ = goal.gripper_pose;
      //transformGripperPose();
      gripper_opening_ = goal.gripper_opening;
      updateGripperAngle();
    }
    if (get_pose_server_.isPreemptRequested())
    {
      if(test_pose_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE ||
         test_pose_client_.getState() == actionlib::SimpleClientGoalState::PENDING )
      {
        test_pose_client_.cancelGoal();
      }
      get_pose_server_.setPreempted();
      setIdle();
      return;
    }

    pose_state_ = UNTESTED;
    initMarkers();
    pr2_marker_control::TestGripperPoseGoal test_goal;
    test_goal.gripper_pose = gripper_pose_;
    test_goal.gripper_opening = gripper_opening_;
    test_pose_client_.sendGoal( test_goal, boost::bind(&HapticGripper::testGripperResultCallback, this, _1, _2));
  }


  //! Callback to allow this action to get preempted by backend.
  void preemptCB()
  {
    ROS_INFO("%s: Preempted", get_pose_name_.c_str());
    test_pose_client_.cancelGoal();
    grasp_plan_client_.cancelGoal();
    get_pose_server_.setPreempted();
    setIdle();
  }

  //! Translate the control pose to the wrist.
  geometry_msgs::PoseStamped toWrist(const geometry_msgs::PoseStamped &ps)
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
  geometry_msgs::PoseStamped fromWrist(const geometry_msgs::PoseStamped &ps)
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
  void updatePoses()
  {
    server_.setPose("ghosted_gripper", gripper_pose_.pose, gripper_pose_.header);
    server_.setPose("object_cloud", gripper_pose_.pose, gripper_pose_.header);
  }

  geometry_msgs::PoseStamped getDefaultPose(std::string arm_name)
  {
    ros::Time now = ros::Time(0);
    tfl_.waitForTransform("r_gripper_tool_frame","/base_link", now, ros::Duration(2.0));
    tf::StampedTransform stamped;
    geometry_msgs::TransformStamped ts;
    geometry_msgs::PoseStamped ps;
    std::string arm_frame;
    if (arm_name == "right_arm") arm_frame="r_wrist_roll_link";
    else if (arm_name == "left_arm") arm_frame="l_wrist_roll_link";
    else
    {
      arm_frame="r_wrist_roll_link";
      ROS_ERROR("Unknown arm name passed to ghosted gripper");
    }
    tfl_.lookupTransform("/base_link", arm_frame, now, stamped);
    tf::transformStampedTFToMsg(stamped, ts);
    ps = msg::createPoseStampedMsg(ts);
    return ps;
  }


// **** 10 ***** 20 ****** 30 ****** 40 ****** 50 ****** 60 ****** 70 ****** 80 ****** 90 ***** 100 ***** 110 ***** 120

  /** \brief Re-initializes all markers.
    */
  void initMarkers()
  {
    initHapticGripper();
    initObjectMarker();
    initGripperControl();
  }

  void initButtonMarker()
  {
    button_marker_pose_.header.stamp = ros::Time(0);
    int num = 0;
    if(planner_poses_.size()) num = planner_index_ + 1;
    server_.insert(makeListControl("grasp_toggle", button_marker_pose_, num, planner_poses_.size(), 0.3));
    //               , boost::bind( &HapticGripper::cycleGrasps, this));
    server_.setCallback("grasp_toggle", boost::bind( &HapticGripper::cycleGrasps, this, _1),
                        visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN );
  }

  void initObjectMarker()
  {
    if(object_cloud_->points.size())
    {
      server_.insert(makeCloudMarker( "object_cloud", gripper_pose_, 0.004, object_manipulator::msg::createColorMsg(0.2, 0.8, 0.2,1.0) ));
    }
  }

  void initHapticGripper()
  {
    float r,g,b;
    switch(pose_state_)
    {
    case INVALID:
      r = 1.0; g = 0.2; b = 0.2;
      break;
    case VALID:
      r = 0.2; g = 1.0; b = 0.2;
      break;
    default:
      r = 0.5; g = 0.5; b = 0.5;
    }
    std_msgs::ColorRGBA color = object_manipulator::msg::createColorMsg(r,g,b,1.0);

    server_.insert(makeGripperMarker( "ghosted_gripper", gripper_pose_, 0.18, color, gripper_angle_, false),
                   boost::bind( &HapticGripper::gripperClickCB, this, _1));
    menu_gripper_.apply(server_, "ghosted_gripper");
  }

  void initGripperControl()
  {
    geometry_msgs::PoseStamped ps = fromWrist(gripper_pose_);
    ps.header.stamp = ros::Time(0);
    server_.insert(make6DofMarker( "gripper_controls", ps, 0.2, false, false),
                   boost::bind( &HapticGripper::updateGripper, this, _1));
    menu_gripper_.apply(server_, "gripper_controls");
  }


protected:


  //! ROS spin update callback
  void spinOnce()
  {
    //updatePoses();
    server_.applyChanges();
  }

  void slowSync()
  {
  }

  //! Callback that receives the result of a TestGripperPose action.
  void testGripperResultCallback(const actionlib::SimpleClientGoalState& state,
                                 const pr2_marker_control::TestGripperPoseResultConstPtr &result)
  {
    if(state.state_ == state.SUCCEEDED)
    {
      ROS_DEBUG("Test pose action returned with result %d", (int)result->valid);
      if (result->valid)
      {
        pose_state_ = VALID;
        if(planner_states_.size())  planner_states_[planner_index_] = VALID;
      }
      else
      {
        pose_state_ = INVALID;
        if(planner_states_.size())  planner_states_[planner_index_] = INVALID;
      }
      initHapticGripper();
    }
    else
    {
      ROS_WARN("Test pose action did not succeed; state = %d", (int)state.state_);
    }
  }

  //! Return with the gripper pose if the pose is valid, otherwise do nothing
  void graspPlanCB()
  {
    planner_index_ = 0;
    planner_poses_.resize(0);
    planner_states_.resize(0);

    point_cloud_server::StoreCloudGoal cloud_goal;
    cloud_goal.action = cloud_goal.GET;
    cloud_goal.name = "interactive_manipulation_snapshot";
    cloud_server_client_.sendGoal(cloud_goal);
    if(!cloud_server_client_.waitForResult(ros::Duration(3.0)))
    {
      ROS_WARN("Timed-out while waiting for cloud from server!");
      return;
    }

    object_manipulation_msgs::GraspPlanningGoal plan_goal;
    plan_goal.target.region.cloud = cloud_server_client_.getResult()->cloud;
    int cloud_size = plan_goal.target.region.cloud.width * plan_goal.target.region.cloud.height;
    plan_goal.target.reference_frame_id = gripper_pose_.header.frame_id;
    object_manipulation_msgs::Grasp seed;
    seed.grasp_pose = gripper_pose_.pose;
    plan_goal.grasps_to_evaluate.push_back(seed);
    ROS_DEBUG_STREAM("Requesting adjustment on cloud with " << cloud_size << " points, pose \n" << seed);
    //pub_cloud_.publish(plan_goal.target.region.cloud);
    grasp_plan_client_.sendGoal( plan_goal, boost::bind(&HapticGripper::graspPlanResultCB, this, _1, _2));

    button_marker_pose_ = gripper_pose_;
    button_marker_pose_.header.stamp = ros::Time(0);
    button_marker_pose_.pose.orientation = geometry_msgs::Quaternion();
    button_marker_pose_.pose.orientation.w = 1;
    button_marker_pose_.pose.position.z -= 0.2;
    initButtonMarker();
    //if(always_call_planner)
    //{
    server_.erase("gripper_controls");
    server_.erase("ghosted_gripper");
    //}
  }

  //! Callback that receives the result of a TestGripperPose action.
  void graspPlanResultCB(const actionlib::SimpleClientGoalState& state,
                         const object_manipulation_msgs::GraspPlanningResultConstPtr &result)
  {
    planner_index_ = 0;
    if(state.state_ == state.SUCCEEDED)
    {
      ROS_INFO("Grasp plan action succeeded.");

      int num = result->grasps.size();

      planner_poses_.resize(num + 1);
      planner_states_.resize(num + 1);
      //gripper_score_.resize(num);
      //gripper_angle_.resize(num);

      for(int i = 0; i < num; i++)
      {
        planner_poses_[i].pose = result->grasps[i].grasp_pose;
        planner_poses_[i].header = gripper_pose_.header;
        planner_poses_[i].header.stamp = ros::Time(0);
        planner_states_[i] = UNTESTED;
        //object_manipulation_msgs::Grasp grasp = result->grasps[i];
        //gripper_angle_[i] = std::max( std::min(1.f , (float)0.541) , (float)0.0);
        //gripper_score_[i] = grasp.success_probability;
      }
      planner_poses_[num] = gripper_pose_;
      planner_poses_[num].header.stamp = ros::Time(0);
      planner_states_[num] = UNTESTED;

      gripper_pose_ = planner_poses_[0];
      //ROS_INFO_STREAM("Result was \n" << gripper_pose_);
      //transformGripperPose();
      //ROS_INFO_STREAM("But after transform \n" << gripper_pose_);
      pose_state_ = UNTESTED;
      initMarkers();
      initButtonMarker();
      testPose();
    }
    else
    {
      ROS_WARN("Grasp plan action did not succeed; state = %d", (int)state.state_);
      planner_poses_.resize(1);
      planner_states_.resize(1);
      planner_poses_[0] = gripper_pose_;
      planner_poses_[0].header.stamp = ros::Time(0);
      planner_states_[0] = UNTESTED;
      gripper_pose_ = planner_poses_[0];
      pose_state_ = UNTESTED;
      initMarkers();
      initButtonMarker();
      testPose();
    }
  }

  void cycleGrasps(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    //ROS_WARN("Grasp cycling not yet implemented!");
    // Need to:
    // Store the returned grasp list
    // Store a bool list to remeber if a grasp pose is valid
    // Set the gripper opening to max (?)
    // Cycle through the grasps, calling the test_pose action if needed
    //ROS_INFO_STREAM("Got feedback! \n" << *feedback);

    //if(!feedback->control_name.compare("left")) planner_index_ --;
    //if(!feedback->control_name.compare("right")) planner_index_++;

    planner_index_++;
    if( planner_index_ == (int)planner_poses_.size() )  planner_index_ = 0;
    //if( planner_index_ < 0 )  planner_index_ = 0;

    gripper_pose_ = planner_poses_[ planner_index_ ];
    pose_state_ = planner_states_[ planner_index_ ];
    initMarkers();
    initButtonMarker();
    if(pose_state_ == UNTESTED) testPose();
  }

  //! Return with the gripper pose if the pose is valid, otherwise do nothing
  void acceptCB()
  {
    if( pose_state_ == VALID )
    {
      pr2_marker_control::GetGripperPoseResult result;
      result.gripper_pose = gripper_pose_;
      result.gripper_opening = gripper_opening_;
      get_pose_server_.setSucceeded(result);
      setIdle();
    }
  }

  //! Cancel this action call
  void cancelCB()
  {
    if(test_pose_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE ||
       test_pose_client_.getState() == actionlib::SimpleClientGoalState::PENDING )
    {
      test_pose_client_.cancelGoal();
    }
    get_pose_server_.setAborted();
    setIdle();
  }

  //! Called when the gripper is clicked; each call cycles through gripper opening values.
  void gripperClickCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    ros::Time now = ros::Time(0);

    switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        //ROS_DEBUG_STREAM( "Marker is being moved, stored pose is invalidated." );
        float max_gripper_angle = 0.541;
        gripper_angle_ -= 0.12;
        if(gripper_angle_ < 0.04)
          gripper_angle_ = max_gripper_angle;
        updateGripperOpening();
        initHapticGripper();
        ROS_DEBUG( "Gripper opening = %.2f, angle = %.2f", gripper_opening_, gripper_angle_);
        break;
    }
  }

  void testPose()
  {
    pr2_marker_control::TestGripperPoseGoal goal;
    goal.gripper_pose = gripper_pose_;
    goal.gripper_opening = gripper_opening_;
    test_pose_client_.sendGoal( goal, boost::bind(&HapticGripper::testGripperResultCallback, this, _1, _2));
    pose_state_ = UNTESTED;
  }

  //! Callback for pose updates from the controls.
  void updateGripper( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    ros::Time now = ros::Time(0);

    switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        ROS_DEBUG_STREAM( "Marker is being moved, stored pose is invalidated." );
        test_pose_client_.cancelAllGoals();
        pose_state_ = UNTESTED;
        break;
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        //ROS_INFO_STREAM("MOUSE_UP \n" << feedback->pose);
        ROS_DEBUG_STREAM( "Marker was released, storing pose and checking." );
//        if(gripper_pose_.header.frame_id.compare(feedback->header.frame_id))  fix = true;
        gripper_pose_.pose = feedback->pose;
        gripper_pose_.header = feedback->header;
        gripper_pose_ = toWrist(gripper_pose_);
        //transformGripperPose();
        //ROS_INFO_STREAM("MOUSE_UP \n" << gripper_pose_);
        initMarkers();
        testPose();
        break;
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        ROS_DEBUG_STREAM("POSE_UPDATE in frame " << feedback->header.frame_id << std::endl << feedback->pose);
        //if(gripper_pose_.header.frame_id.compare(feedback->header.frame_id))  fix = true;
        gripper_pose_.pose = feedback->pose;
        gripper_pose_.header = feedback->header;
        gripper_pose_ = toWrist(gripper_pose_);
        updatePoses();
        break;
    }
  }

  //! Initialize the menus for all markers.
  void initMenus()
  {
    accept_handle_ = menu_gripper_.insert("Accept", boost::bind( &HapticGripper::acceptCB, this ) );
    cancel_handle_ = menu_gripper_.insert("Cancel", boost::bind( &HapticGripper::cancelCB, this ) );
  }

  bool getModelMesh( int model_id, arm_navigation_msgs::Shape& mesh )
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
  visualization_msgs::InteractiveMarker makeCloudMarker( const char *name, const geometry_msgs::PoseStamped &stamped,
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

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "haptic_gripper");
  HapticGripper haptic_gripper;

  ros::Duration(1.0).sleep();

  ros::spin();
}
