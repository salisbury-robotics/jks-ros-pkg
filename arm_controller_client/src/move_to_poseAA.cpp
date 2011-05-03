#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <boost/thread.hpp>
#include <kinematics_msgs/GetPositionIK.h>
#include <tf/tf.h>
#include <motion_planning_msgs/FilterJointTrajectory.h>

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
  double m[16];
  T.getOpenGLMatrix(m);

  tf::Vector3 x(m[0], m[1], m[2]);
  tf::Vector3 P(req.ik_request.pose_stamped.pose.position.x,
                req.ik_request.pose_stamped.pose.position.y,
                req.ik_request.pose_stamped.pose.position.z );
  tf::Vector3 newPosition = P - 0.10*x; // was 0.18
  req.ik_request.pose_stamped.pose.position.x = newPosition.x();
  req.ik_request.pose_stamped.pose.position.y = newPosition.y();
  req.ik_request.pose_stamped.pose.position.z = newPosition.z();


    return ik_client.call(req, res);
  }

int main(int argc, char** argv)
{
  if(argc < 8)
    {
    ROS_INFO("Usage:: ./move_to_poseAA target_position target_axis target_angle <L/R>");
    ROS_INFO("Example:: ./move_to_pose 0.5 0 0.1 0 0 1 0 L");
    exit(1);
    }
  ros::init(argc, argv, "talker");
  boost::thread spin_thread(&spinThread);
  ros::NodeHandle n;
  ros::Publisher arm_pub;
  
  bool useRightArm;
  if( *argv[8] == 'R' ) useRightArm = true;
  else                  useRightArm = false;
  // Define the joint names
  std::vector<std::string> joint_names;
  joint_names.resize(7);
  
  if( useRightArm )
  {
    joint_names[0] = "r_shoulder_pan_joint";
    joint_names[1] = "r_shoulder_lift_joint";
    joint_names[2] = "r_upper_arm_roll_joint";
    joint_names[3] = "r_elbow_flex_joint";
    joint_names[4] = "r_forearm_roll_joint";
    joint_names[5] = "r_wrist_flex_joint";
    joint_names[6] = "r_wrist_roll_joint";
    arm_pub = n.advertise<trajectory_msgs::JointTrajectory>("r_arm_controller/command", 100);
  } else {
    joint_names[0] = "l_shoulder_pan_joint";
    joint_names[1] = "l_shoulder_lift_joint";
    joint_names[2] = "l_upper_arm_roll_joint";
    joint_names[3] = "l_elbow_flex_joint";
    joint_names[4] = "l_forearm_roll_joint";
    joint_names[5] = "l_wrist_flex_joint";
    joint_names[6] = "l_wrist_roll_joint";
    arm_pub = n.advertise<trajectory_msgs::JointTrajectory>("l_arm_controller/command", 100);
  }
  
  while(arm_pub.getNumSubscribers() < 1 && n.ok())
    ros::Duration(0.1).sleep();
  trajectory_msgs::JointTrajectory trajectory;

  //Add filter stuff
  ros::service::waitForService(FILTER_SERVICE);
  motion_planning_msgs::FilterJointTrajectory::Request traj_req;
  motion_planning_msgs::FilterJointTrajectory::Response traj_res;
  ros::ServiceClient filter_trajectory_client_ = n.serviceClient<motion_planning_msgs::FilterJointTrajectory>(FILTER_SERVICE);


  // Add the IK stuff
  if( useRightArm )
  {
    ros::service::waitForService("pr2_right_arm_kinematics/get_ik");
    ik_client = n.serviceClient<kinematics_msgs::GetPositionIK>("pr2_right_arm_kinematics/get_ik");
  } else {
    ros::service::waitForService("pr2_left_arm_kinematics/get_ik");
    ik_client = n.serviceClient<kinematics_msgs::GetPositionIK>("pr2_left_arm_kinematics/get_ik");
  }
  
  // define the service messages
  kinematics_msgs::GetPositionIK::Request  req;
  kinematics_msgs::GetPositionIK::Response res;



  // initialize the request
  req.timeout = ros::Duration(2.0);
  if( useRightArm )
  {
    req.ik_request.ik_link_name = "r_wrist_roll_link";
  } else {
    req.ik_request.ik_link_name = "l_wrist_roll_link";
  }
  
  req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";  // Specify the frame in which the pose is specified
  // desired pose
  ROS_INFO("GettingIK...");
  req.ik_request.pose_stamped.pose.position.x =     atof(argv[1]);
  req.ik_request.pose_stamped.pose.position.y =     atof(argv[2]);
  req.ik_request.pose_stamped.pose.position.z =     atof(argv[3]);
  tf::Quaternion trans(tf::Vector3(atof(argv[4]), atof(argv[5]), atof(argv[6])), atof(argv[7])*M_PI/180.0);
  req.ik_request.pose_stamped.pose.orientation.x =  trans.x();
  req.ik_request.pose_stamped.pose.orientation.y =  trans.y();
  req.ik_request.pose_stamped.pose.orientation.z =  trans.z();
  req.ik_request.pose_stamped.pose.orientation.w =  trans.w();
  // The IK request always takes a seed state which must contain the joints in the chain
  req.ik_request.ik_seed_state.joint_state.position.resize(joint_names.size());
  req.ik_request.ik_seed_state.joint_state.name = joint_names;
  for(unsigned int i=0; i< joint_names.size(); i++)
    req.ik_request.ik_seed_state.joint_state.position[i] = 0.0;

  if(myIK(req, res))
  //if(ik_client.call(req, res))
  {
    if(res.error_code.val == res.error_code.SUCCESS)
    {
      for(unsigned int i=0; i < res.solution.joint_state.name.size(); i ++)
      {
        ROS_INFO("Joint: %s %f",res.solution.joint_state.name[i].c_str(),res.solution.joint_state.position[i]);
        //fk_request.robot_state.joint_state.position[i] = res.solution.joint_state.position[i];
      }
    }
    else
    {
      ROS_ERROR("Inverse kinematics failed");
    }
  }
  else
    ROS_ERROR("Inverse kinematics service call failed");

  // put IK solution into trajectory!
  trajectory.joint_names = joint_names;

  trajectory.points.resize(1);  // Define the joint names
  trajectory.points[0].positions.resize(joint_names.size());
  trajectory.points[0].positions = res.solution.joint_state.position;
  trajectory.points[0].time_from_start = ros::Duration(2.0);

  traj_req.trajectory.joint_names = joint_names;
  traj_req.trajectory.points = trajectory.points;
  traj_req.allowed_time = ros::Duration(10.0);

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

  trajectory = traj_res.trajectory;
  trajectory.header.stamp = ros::Time::now();
  arm_pub.publish(trajectory);
  ros::shutdown();
  spin_thread.join();

}
