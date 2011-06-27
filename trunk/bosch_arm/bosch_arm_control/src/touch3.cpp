/** A simple joint1 controller
    It stops upon touching the obstacle.

    service is replaced by topic to achieve higher command rate.
    about 10% command jitters.
**/
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <boost/lexical_cast.hpp>
#include <bosch_arm_control/PointCommand.h>

using namespace std;
ros::Publisher joint_cmd_pub;
sensor_msgs::JointState js;
void jointStateCallBack ( const sensor_msgs::JointState& msg )
{
  js=msg;
}
int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "touch_stop" );
  ros::NodeHandle n;
  ros::Rate loop_rate ( 800 );
  double acc=boost::lexical_cast<double> ( argv[1] );
  double threshold=boost::lexical_cast<double> ( argv[2] );
  double adjust=boost::lexical_cast<double> ( argv[3] );
  double nacc=boost::lexical_cast<int> ( argv[4] );
  joint_cmd_pub =  n.advertise<bosch_arm_control::PointCommand> ( "/single_point_cmd",100 );
  ros::Subscriber joint_state_sub = n.subscribe ( "/joint_states", 100, jointStateCallBack );
  bosch_arm_control::PointCommand cmd;
  cmd.joint_angles.resize ( 4 );
  cmd.header.frame_id="";
  //get current joint angles
  while ( js.position.size() ==0 )
    ros::spinOnce();
  //estimate current command position
  for ( int i=0;i<4;i++ )
    cmd.joint_angles[i]=js.position[i];  
  cmd.joint_angles[0]+=adjust;
  //initialize topic connection
  for(int i=0;i<10;i++)
  {
    cmd.header.stamp=ros::Time::now();
    joint_cmd_pub.publish ( cmd );
  }
  
  double last_cmd=cmd.joint_angles[0];
  ros::Time tlast=cmd.header.stamp;
  //adjust=cmd.joint_angles[0]-js.position[0];
  double q1err;
  int nstep=1;
  
  
  bool touched=false;
  double final_pos;
  while ( ros::ok() )
  {
    ros::Time tnow;
    tnow=ros::Time::now();
    cmd.header.stamp=tnow;
    joint_cmd_pub.publish ( cmd );
    //ros::Duration dura=tnow-tlast;
    //printf("%d,%d\n",dura.sec,dura.nsec);
    tlast=tnow;
    loop_rate.sleep();
    //get the response of the last command
    ros::spinOnce();
    
    if ( touched )
    {
      if ( cmd.joint_angles[0]>=js.position[0]+adjust )
        break;
      //gradually reduce pushing force
      cmd.joint_angles[0]-=nstep*acc;
      //printf("%f,%f\n",js.position[0],cmd.joint_angles[0]);
    }
    else
    {
      q1err=js.position[0]-last_cmd;
      if(q1err>threshold)
      {
        final_pos=js.position[0];
        touched=true;
      }
      //save current command to compare with the next get
      last_cmd=cmd.joint_angles[0];
      //compute new command for the next set
      if ( nstep<nacc )
      {
        cmd.joint_angles[0]+=nstep*acc;
        ++nstep;
      }
      else
        cmd.joint_angles[0]+=nstep*acc;
    }

  }
  return 0;
}
