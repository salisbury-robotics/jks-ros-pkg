/** A simple joint1 controller
    It stops upon touching the obstacle.

    service is replaced by topic to achieve higher command rate.
    about 10% command jitters.
**/
#include "ros/ros.h"
#include <bosch_arm_control/TipState.h>
#include <boost/lexical_cast.hpp>
#include <bosch_arm_control/PointCmd.h>

using namespace std;
ros::Publisher joint_cmd_pub;
bosch_arm_control::TipState js;
void jointStateCallBack ( const bosch_arm_control::TipState& msg )
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
  //double adjust=boost::lexical_cast<double> ( argv[3] );
  double nacc=boost::lexical_cast<int> ( argv[3] );
  joint_cmd_pub =  n.advertise<bosch_arm_control::PointCmd> ( "/cartesian_point_cmd",100 );
  ros::Subscriber joint_state_sub = n.subscribe ( "/tip_states", 100, jointStateCallBack );
  bosch_arm_control::PointCmd cmd;
  cmd.coordinates.resize ( 3 );
  cmd.header.frame_id="";
  //get current joint angles
  while ( js.position.size() ==0 )
    ros::spinOnce();
  //estimate current command position
  for ( int i=0;i<3;i++ )
    cmd.coordinates[i]=js.position[i];  
  //cmd.coordinates[0]+=adjust;
  //initialize topic connection
  for(int i=0;i<10;i++)
  {
    cmd.header.stamp=ros::Time::now();
    joint_cmd_pub.publish ( cmd );
  }
  
  vector<double> last_cmd=cmd.coordinates;
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
      if ( cmd.coordinates[2]>=js.position[2] )
        break;
      //gradually reduce pushing force
      cmd.coordinates[0]-=nstep*acc;
      cmd.coordinates[2]+=nstep*2*acc;
      //printf("%f,%f\n",js.position[0],cmd.joint_angles[0]);
    }
    else
    {
      q1err=js.position[2]-last_cmd[2];
      if(q1err>threshold)
      {
        final_pos=js.position[2];
        touched=true;
      }
      //save current command to compare with the next get
      last_cmd=cmd.coordinates;
      //compute new command for the next set
      if ( nstep<nacc )
      {
        cmd.coordinates[0]+=nstep*acc;
        cmd.coordinates[2]-=nstep*2*acc;
        ++nstep;
      }
      else{
        cmd.coordinates[0]+=nstep*acc;
        cmd.coordinates[2]-=nstep*2*acc;
      }
    }

  }
  return 0;
}
