/** A simple cartesian space controller.
    It receives a destination, a duration and publishes the trajectory
    in cartesian space at 200Hz
    Note: the robot is required to be calibrated to the zero configuration
    as used in the kdl chain model.
**/
#include "ros/ros.h"
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include "bosch_arm_srvs/GetJointAngles.h"
int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "traj_gen" );
  ros::NodeHandle n;
  ros::Publisher traj_pub =  n.advertise<trajectory_msgs::JointTrajectory> ( "/traj_cmd",1 );
  //boost::thread t2 = boost::thread::thread ( boost::bind ( &pubTrajectory ) );
  ros::ServiceClient client = n.serviceClient<bosch_arm_srvs::GetJointAngles>("get_joint_angles");
  bosch_arm_srvs::GetJointAngles srv;
  client.call(srv);
  
  //read destination and duration
  double xyz[3];
  for(int i=0;i<3;i++)
    xyz[i]=boost::lexical_cast<double>(argv[i+1]);
  double t=boost::lexical_cast<double>(argv[4]);
  int npts=ceil(t*200);
  
  //TODO convert destination to joint angles
  double des[4];
  //linear interpolation in joint space
  trajectory_msgs::JointTrajectory traj;
  traj.header.stamp=ros::Time::now();
  traj.header.frame_id="";
  traj.header.seq=0;
  traj.points.resize(npts+1);
  traj.joint_names.resize ( 4 );
  traj.joint_names[0]="joint1";
  traj.joint_names[1]="joint2";
  traj.joint_names[2]="joint3";
  traj.joint_names[3]="joint4";
  double dq[4];
  for(int i=0;i<4;i++)
    dq[i]=(des[i]-srv.response.joint_angles[i])/npts;
  for(int i=0;i<=npts;i++)
  {
    traj.points[i].positions.resize(4);
    for(int j=0;j<4;j++)
      traj.points[i].positions[j]=srv.response.joint_angles[j]+i*dq[j];
    traj.points[i].time_from_start=ros::Duration(i*t/npts);
  }
  //The first message is always lost
  for(int i=0;i<2;i++){
  traj_pub.publish(traj);
  sleep(1);
  }
  ros::spin();
  return 0;
}