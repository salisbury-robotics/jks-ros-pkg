#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <boost/thread.hpp>


void spinThread()
{
  ros::spin();
}

int main(int argc, char** argv)
{
  if(argc < 2)
    {
    ROS_INFO("Usage:: ./arm_controller_client r_shoulder_pan_joint_target_position");
    ROS_INFO("Example:: ./arm_controller_client -1.0");
    exit(1);
    }
  ros::init(argc, argv, "talker");
  boost::thread spin_thread(&spinThread);
  ros::NodeHandle n;
  ros::Publisher arm_pub = n.advertise<trajectory_msgs::JointTrajectory>("r_arm_controller/command", 100);
  while(arm_pub.getNumSubscribers() < 1 && n.ok())
    ros::Duration(0.1).sleep();
  trajectory_msgs::JointTrajectory trajectory;

  trajectory.joint_names.resize(7);
  trajectory.joint_names[0] = "r_shoulder_pan_joint";
  trajectory.joint_names[1] = "r_shoulder_lift_joint";
  trajectory.joint_names[2] = "r_upper_arm_roll_joint";
  trajectory.joint_names[3] = "r_elbow_flex_joint";
  trajectory.joint_names[4] = "r_forearm_roll_joint";
  trajectory.joint_names[5] = "r_wrist_flex_joint";
  trajectory.joint_names[6] = "r_wrist_roll_joint";

  trajectory.points.resize(1);
  trajectory.points[0].positions.resize(7);
  trajectory.points[0].positions[0] = atof(argv[1]);
  trajectory.points[0].positions[6] = atof(argv[2]);
  trajectory.points[0].time_from_start = ros::Duration(4.0);

  trajectory.header.stamp = ros::Time::now();
  arm_pub.publish(trajectory);
  ros::shutdown();
  spin_thread.join();

}
