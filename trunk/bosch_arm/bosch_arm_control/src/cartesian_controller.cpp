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
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <vector>
using namespace Eigen;

class BoschArmKinematicModel
{
  public:
    double L0,L3,L4;
    Vector3f getTipPosition(Vector4f q)
    {
      Vector3f tip;
      double q1=q(0);
      double q2=q(1);
      double q3=q(2);
      double q4=q(3);
      tip(0)= - L4*(sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) + cos(q2)*cos(q4)*sin(q1)) - L3*cos(q2)*sin(q1);
      tip(1)=  - L4*(cos(q4)*sin(q2) + cos(q2)*sin(q3)*sin(q4)) - L3*sin(q2);
      tip(2)=L0 - L4*(sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - cos(q1)*cos(q2)*cos(q4)) + L3*cos(q1)*cos(q2);
      return tip;
    }
    Matrix3f getJacobianLockJoint3(Vector4f qlock)
    {
      Matrix3f jacob;
      double q1=qlock(0);
      double q2=qlock(1);
      double q3=qlock(2);
      double q4=qlock(3);
      jacob(0,0)=L4*(sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - cos(q1)*cos(q2)*cos(q4)) - L3*cos(q1)*cos(q2);
      jacob(0,1)=L4*(cos(q4)*sin(q1)*sin(q2) + cos(q2)*sin(q1)*sin(q3)*sin(q4)) + L3*sin(q1)*sin(q2);
      jacob(0,2)=-L4*(cos(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) - cos(q2)*sin(q1)*sin(q4));
      jacob(1,0)=0;
      jacob(1,1)=-L4*(cos(q2)*cos(q4) - sin(q2)*sin(q3)*sin(q4)) - L3*cos(q2);
      jacob(1,2)=L4*(sin(q2)*sin(q4) - cos(q2)*cos(q4)*sin(q3));
      jacob(2,0)=- L4*(sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) + cos(q2)*cos(q4)*sin(q1)) - L3*cos(q2)*sin(q1);
      jacob(2,1)=- L4*(cos(q1)*cos(q4)*sin(q2) + cos(q1)*cos(q2)*sin(q3)*sin(q4)) - L3*cos(q1)*sin(q2);
      jacob(2,2)=-L4*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) + cos(q1)*cos(q2)*sin(q4));
      return jacob;
    }

    
};
int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "traj_gen" );
  ros::NodeHandle n;
  ros::Publisher traj_pub =  n.advertise<trajectory_msgs::JointTrajectory> ( "/traj_cmd",1 );
  //boost::thread t2 = boost::thread::thread ( boost::bind ( &pubTrajectory ) );
  ros::ServiceClient client = n.serviceClient<bosch_arm_srvs::GetJointAngles> ( "get_joint_angles" );
  bosch_arm_srvs::GetJointAngles srv;
  client.call ( srv );

  //read destination and duration
  KDL::Vector des;
  for ( int i=0;i<3;i++ )
    des[i]=boost::lexical_cast<double> ( argv[i+1] );
  double t=boost::lexical_cast<double> ( argv[4] );


  //TODO convert destination to joint angles
  
  Segment seg0=Segment ( Joint ( Joint::None ),
                         Frame ( Rotation::RPY ( M_PI/2,-M_PI/2,0 ),Vector ( 0,0,0.27 ) ) );
  Segment seg1=Segment ( Joint ( Joint::RotZ ),
                         Frame ( Rotation::RPY ( M_PI/2,-M_PI/2,0 ) ) );
  Segment seg2=Segment ( Joint ( Joint::RotZ ),
                         Frame ( Rotation::RPY ( M_PI/2,-M_PI/2,0 ) ) );
  Segment seg3=Segment ( Joint ( Joint::None ),
                         Frame ( Rotation::RPY ( M_PI/2,-M_PI/2,0 ),Vector ( 0,0,0.50 ) ) );
  Segment seg4=Segment ( Joint ( Joint::RotZ ),
                         Frame ( Vector ( 0.48,0,0 ) ) );
  Chain chain;
  chain.addSegment ( seg0 );
  chain.addSegment ( seg1 );
  chain.addSegment ( seg2 );
  chain.addSegment ( seg3 );
  chain.addSegment ( seg4 );
  ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive ( chain );

  //linear interpolation in joint space
  trajectory_msgs::JointTrajectory traj;
  traj.header.stamp=ros::Time::now();
  traj.header.frame_id="";
  traj.header.seq=0;
  traj.points.resize ( npts+1 );
  traj.joint_names.resize ( 4 );
  traj.joint_names[0]="joint1";
  traj.joint_names[1]="joint2";
  traj.joint_names[2]="joint3";
  traj.joint_names[3]="joint4";
  traj.points[i].positions.resize ( 4 );

  int npts=ceil ( t*200 );
  KDL::JntArray jointpositions = JntArray (3);
  KDL::Frame cartpos;
  double q3;
  bool kinematics_status;
  
  for ( int i=0;i<=npts;i++ )
  {
    //get the current joint position
    client.call ( srv );
    //compute forward kinematics.
    
    jointpositions (0) =srv.response.joint_angles[0];
    jointpositions (1) =srv.response.joint_angles[1];
    q3                 =srv.response.joint_angles[2];
    jointpositions (2) =srv.response.joint_angles[3]; 
    kinematics_status = fksolver.JntToCart ( jointpositions,cartpos );
    if ( kinematics_status>=0 )
    {
      std::cout << cartpos.p <<std::endl;
    }
    else
    {
      printf ( "%s \n","Error: could not calculate forward kinematics :(" );
    }
    double steps_to_go=npts-i;
    Vector step_length_cart= (des-cartpos.p)/steps_to_go;
    
    for ( int j=0;j<4;j++ )
      traj.points[i].positions[j]=srv.response.joint_angles[j]+i*dq[j];
    traj.points[i].time_from_start=ros::Duration ( i*t/npts );
  }
  //The first message is always lost
  for ( int i=0;i<2;i++ )
  {
    traj_pub.publish ( traj );
    sleep ( 1 );
  }
  ros::spin();
  return 0;
}
