/** A simple joint1 controller
    It stops upon touching the obstacle.
    
    use another trajectory that is smooth at start.
**/
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <unistd.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <netdb.h>              //gethostbyname
#include <netinet/in.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <bosch_arm_control/PointCommand.h>
//#include "bosch_arm_srvs/GetJointAngles.h"
//#include "bosch_arm_srvs/SetJointAngles.h"
using namespace std;
ros::Publisher joint_cmd_pub;
sensor_msgs::JointState js;
void jointStateCallBack(const sensor_msgs::JointState& msg)
{
  js=msg;
}
int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "touch_stop" );
  ros::NodeHandle n;
  double acc=boost::lexical_cast<double> ( argv[1] );
  double threshold=boost::lexical_cast<double> ( argv[2] );
  double adjust=boost::lexical_cast<double> ( argv[3] );
  double nacc=boost::lexical_cast<int> ( argv[4] );
  joint_cmd_pub =  n.advertise<bosch_arm_control::PointCommand> ( "/single_point_cmd",100 );
  ros::Subscriber joint_state_sub = n.subscribe ( "/joint_states", 3, jointStateCallBack );
  bosch_arm_control::PointCommand cmd;
  cmd.joint_angles.resize(4);
  cmd.header.frame_id="";
  //initial get
  while(js.position.size()==0)
    ros::spinOnce();
  //initial set
  for(int i=0;i<4;i++)
    cmd.joint_angles[i]=js.position[i];
  cmd.header.stamp=ros::Time::now();
  joint_cmd_pub.publish(cmd);

  ros::Rate loop_rate ( 800 );
  
  double q1err;
  int nstep=1;
  //timespec tlast;
  ros::Time tlast=cmd.header.stamp;
  //tlast.tv_nsec=0;
  //tlast.tv_sec=0;
  double last_cmd=cmd.joint_angles[0];
//  int64_t period=1250000;
  while ( ros::ok() )
  {      
    
    //timespec tnow;
    ros::Time tnow;
    //int64_t diff_nsec;
    //wait for 2.5ms
//     do
//     {
//       clock_gettime(CLOCK_REALTIME, &tnow); 
//       diff_nsec=(tnow.tv_sec-tlast.tv_sec)*1000000000L+(tnow.tv_nsec-tlast.tv_nsec);
//     }   
//     while(diff_nsec<period);
    //printf("%ld\n",diff_nsec);
    tnow=ros::Time::now();   
    cmd.header.stamp=tnow;
    joint_cmd_pub.publish(cmd);
    ros::Duration dura=tnow-tlast;
    printf("%d,%d\n",dura.sec,dura.nsec);
    tlast=tnow;
    loop_rate.sleep();
    //get the response of the last command
    ros::spinOnce();
    q1err=js.position[0]-last_cmd;
    if ( q1err>threshold ){
      //stop pushing
//       do
//       {
//         clock_gettime(CLOCK_REALTIME, &tnow); 
//         diff_nsec=(tnow.tv_sec-tlast.tv_sec)*1000000000+(tnow.tv_nsec-tlast.tv_nsec);
//       }
//       while(diff_nsec<period);
      loop_rate.sleep();
      cmd.joint_angles[0]=js.position[0];
      cmd.joint_angles[0]+=adjust;
      joint_cmd_pub.publish(cmd);
      break;
    }
    //save current command to compare with the next get
    last_cmd=cmd.joint_angles[0];
    //compute new command for the next set
    if(nstep<=nacc)
    {
      cmd.joint_angles[0]+=nstep*acc;
      ++nstep;
    }else
      cmd.joint_angles[0]+=nstep*acc;
    
  }
  return 0;
}
