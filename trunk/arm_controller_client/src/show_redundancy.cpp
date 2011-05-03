#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <boost/thread.hpp>
#include <kinematics_msgs/GetPositionIK.h>
#include <tf/tf.h>
#include <motion_planning_msgs/FilterJointTrajectory.h>
#include <urdf/model.h>

ros::ServiceClient ik_client;
static const std::string FILTER_SERVICE = "trajectory_filter_unnormalizer/filter_trajectory";

void spinThread()
{
  ros::spin();
}

// Using the result from HW2 Exercise 2a, the offset between r_gripper_tool_frame
// and r_wrist_roll_link is 0.18 meters along the X-axis of r_wrist_roll_link.
bool myIK(kinematics_msgs::GetPositionIK::Request  req, kinematics_msgs::GetPositionIK::Response &res){

  // Modify the request by the offset between wrist_roll and tool_frame
  tf::Quaternion Q(req.ik_request.pose_stamped.pose.orientation.x,
                   req.ik_request.pose_stamped.pose.orientation.y,
                   req.ik_request.pose_stamped.pose.orientation.z,
                   req.ik_request.pose_stamped.pose.orientation.w);
  tf::Transform T(Q);
  double m[15];
  T.getOpenGLMatrix(m);

  tf::Vector3 x(m[0], m[1], m[2]);
  tf::Vector3 P(req.ik_request.pose_stamped.pose.position.x,
                req.ik_request.pose_stamped.pose.position.y,
                req.ik_request.pose_stamped.pose.position.z );
  tf::Vector3 newPosition = P - 0.18*x;
  req.ik_request.pose_stamped.pose.position.x = newPosition.x();
  req.ik_request.pose_stamped.pose.position.y = newPosition.y();
  req.ik_request.pose_stamped.pose.position.z = newPosition.z();


    return ik_client.call(req, res);
  }

int main(int argc, char** argv)
{
  if(argc < 2)
    {
    ROS_INFO("Usage:: ./move_through_redundancy [direction] [startDelay]");
    ROS_INFO("Example:: .//move_through_redundancy [0 or 1] 0.5");
    exit(1);
    }
  ros::init(argc, argv, "talker");
  boost::thread spin_thread(&spinThread);
  ros::NodeHandle n;
  ros::Publisher arm_pub = n.advertise<trajectory_msgs::JointTrajectory>("r_arm_controller/command", 100);
  while(arm_pub.getNumSubscribers() < 1 && n.ok())
    ros::Duration(0.1).sleep();
  trajectory_msgs::JointTrajectory trajectory;

  // Get the model and stuff
  std::string robot_desc_string;
  n.param("robot_description", robot_desc_string, std::string());
  urdf::Model model;
  if (!model.initString(robot_desc_string)){
    ROS_ERROR("Failed to parse urdf string");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");

  //Add filter stuff
  ros::service::waitForService(FILTER_SERVICE);
  motion_planning_msgs::FilterJointTrajectory::Request traj_req;
  motion_planning_msgs::FilterJointTrajectory::Response traj_res;
  ros::ServiceClient filter_trajectory_client_ = n.serviceClient<motion_planning_msgs::FilterJointTrajectory>(FILTER_SERVICE);


  // Add the IK stuff
  ros::service::waitForService("pr2_right_arm_kinematics/get_ik");
  ik_client = n.serviceClient<kinematics_msgs::GetPositionIK>("pr2_right_arm_kinematics/get_ik");
  // define the service messages
  kinematics_msgs::GetPositionIK::Request  ik_req;
  kinematics_msgs::GetPositionIK::Response ik_res;

  // Define the joint names
  std::vector<std::string> joint_names;
  joint_names.resize(7);
  joint_names[0] = "r_shoulder_pan_joint";
  joint_names[1] = "r_shoulder_lift_joint";
  joint_names[2] = "r_upper_arm_roll_joint";
  joint_names[3] = "r_elbow_flex_joint";
  joint_names[4] = "r_forearm_roll_joint";
  joint_names[5] = "r_wrist_flex_joint";
  joint_names[6] = "r_wrist_roll_joint";


  // initialize the request
  ik_req.timeout = ros::Duration(5.0);
  ik_req.ik_request.ik_link_name = "r_wrist_roll_link";
  ik_req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";  // Specify the frame in which the pose is specified
  // desired pose
  ROS_INFO("GettingIK...");
  ik_req.ik_request.pose_stamped.pose.position.x = 0.745;
  ik_req.ik_request.pose_stamped.pose.position.y = 0.0;
  ik_req.ik_request.pose_stamped.pose.position.z = 0.05;
  ik_req.ik_request.pose_stamped.pose.orientation.x = 0.0;
  ik_req.ik_request.pose_stamped.pose.orientation.y = 0.0;
  ik_req.ik_request.pose_stamped.pose.orientation.z = 0.0;
  ik_req.ik_request.pose_stamped.pose.orientation.w = 1.0;
//  ik_req.ik_request.pose_stamped.pose.position.x =     atof(argv[1]);
//  ik_req.ik_request.pose_stamped.pose.position.y =     atof(argv[2]);
//  ik_req.ik_request.pose_stamped.pose.position.z =     atof(argv[3]);
//  ik_req.ik_request.pose_stamped.pose.orientation.x =  atof(argv[4]);
//  ik_req.ik_request.pose_stamped.pose.orientation.y =  atof(argv[5]);
//  ik_req.ik_request.pose_stamped.pose.orientation.z =  atof(argv[6]);
//  ik_req.ik_request.pose_stamped.pose.orientation.w =  atof(argv[7]);
  // The IK request always takes a seed state which must contain the joints in the chain
  ik_req.ik_request.ik_seed_state.joint_state.position.resize(joint_names.size());
  ik_req.ik_request.ik_seed_state.joint_state.name = joint_names;
  for(unsigned int i=0; i< joint_names.size(); i++)
    ik_req.ik_request.ik_seed_state.joint_state.position[i] = 0.0;

  // find limits for upper arm roll joint
  boost::shared_ptr<urdf::JointLimits> limits = model.joints_["r_upper_arm_roll_joint"]->limits;
  //ROS_INFO("r_upper_arm_roll_joint: min [%f] max [%f] ", limits->lower, limits->upper);

  // step through the seed for the limits of the upper arm joint
  double seed = limits->lower;
  if(atoi(argv[1]) == 1) seed = limits->upper;

  //double point_time = 0.1;
  //double step = 0.05;
  double point_time = atof(argv[2]);
  double step = 0.1;
  traj_req.trajectory.header.stamp = ros::Time::now();
  for( ; (seed <= limits->upper) && (seed >= limits->lower) ; seed += (atoi(argv[1]) == 1)?(-step):(step))
  {
    ik_req.ik_request.ik_seed_state.joint_state.position[2] = seed;
    if(myIK(ik_req, ik_res))
    //if(ik_client.call(ik_req, ik_res))
    {
//      ROS_INFO("Seed %f", seed);
      if(ik_res.error_code.val == ik_res.error_code.SUCCESS)
      {
        trajectory_msgs::JointTrajectoryPoint point;
        //for(unsigned int i=0; i< joint_names.size(); i++)
        point.positions = ik_res.solution.joint_state.position;
        point_time += 0.2;
        point.time_from_start = ros::Duration(point_time);
        traj_req.trajectory.points.push_back(point);
        ROS_INFO_STREAM("Added point: time = " << traj_req.trajectory.points.back().time_from_start.toSec());
//        for(unsigned int i=0; i < res.solution.joint_state.name.size(); i ++)
//        {
//          //ROS_INFO("Joint: %s %f",res.solution.joint_state.name[i].c_str(),res.solution.joint_state.position[i]);
//          //fk_request.robot_state.joint_state.position[i] = res.solution.joint_state.position[i];
//          trajectory.points.end().position[i] = .joint_state.position[i];
//        }

      }
      else
      {
        ROS_ERROR("Inverse kinematics failed");
      }
    }
    else
      ROS_ERROR("Inverse kinematics service call failed");
  }
  ROS_INFO("traj_req has %d points", (int)traj_req.trajectory.points.size());



  traj_req.trajectory.joint_names = joint_names;
  traj_req.allowed_time = ros::Duration(1.0);

  if(filter_trajectory_client_.call(traj_req, traj_res))
  {
    if(traj_res.error_code.val == traj_res.error_code.SUCCESS)
    {
      ROS_INFO("Requested trajectory was filtered");
      for(unsigned int i=0; i < traj_res.trajectory.points.size(); i++)
      {
        //ROS_INFO_STREAM(traj_res.trajectory.points[i].time_from_start.toSec());
      }
    }
    else
      ROS_INFO("Requested trajectory was not filtered. Error code: %d",traj_res.error_code.val);
  }
  else
  {
    ROS_ERROR("Service call to filter trajectory failed %s",filter_trajectory_client_.getService().c_str());
  }

  ROS_INFO("traj_res has %d points", (int)traj_res.trajectory.points.size());
  trajectory = traj_res.trajectory;
  ROS_INFO("trajectory has %d points", (int)trajectory.points.size());
//  trajectory.points[0].positions.resize(joint_names.size());
//  for(unsigned int i=0; i< joint_names.size(); i++)
//    trajectory.points[0].positions[i] = res.solution.joint_state.position[i];
  //trajectory.points[0].positions[6] = atof(argv[7]);
  //trajectory.points[0].time_from_start = ros::Duration(3.0);

  trajectory.header.stamp = ros::Time::now();
  ROS_INFO("Publishing...");
  arm_pub.publish(trajectory);
  ROS_INFO("done!");
  ros::shutdown();
  spin_thread.join();

}
