/** receives state messages and wrap them as ros JointState messages
    receives command messages and translate them to control commands
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
#include "bosch_arm_srvs/GetJointAngles.h"

ros::Publisher joint_state_pub;
sensor_msgs::JointState js;
void pubJointStates()
{
  int buflen = 512;
  int port = 10050;
  char cmdbuf[512];
  struct sockaddr_in  si_me, si_other;
  int s;
  socklen_t slen = sizeof ( si_other );

  if ( ( s=socket ( AF_INET, SOCK_DGRAM, IPPROTO_UDP ) ) ==-1 )
    ROS_ERROR ( "socket" );

  memset ( ( char * ) &si_me, 0, sizeof ( si_me ) );
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons ( port );
  si_me.sin_addr.s_addr = htonl ( INADDR_ANY );
  if ( bind ( s, ( struct sockaddr * ) &si_me, sizeof ( si_me ) ) ==-1 )
    ROS_ERROR ( "bind" );

  js.name.resize ( 4 );
  js.name[0]="joint1";
  js.name[1]="joint2";
  js.name[2]="joint3";
  js.name[3]="joint4";
  js.header.frame_id="";
  int seq=0;
  for ( ;; )
  {
    memset ( cmdbuf,'\0',buflen );
    int size  = recvfrom ( s, cmdbuf, buflen, 0, ( struct sockaddr * ) &si_other, &slen );
    char* pch;
    pch=strtok ( cmdbuf, "," );
    for ( int i=0;i<16;i++ )
      pch=strtok ( NULL,"," );
    js.header.stamp.sec=boost::lexical_cast<int> ( pch );
    pch=strtok ( NULL,"," );
    js.header.stamp.nsec=boost::lexical_cast<long> ( pch );
    for ( int i=0;i<4;i++ )
      pch=strtok ( NULL,"," );
    js.position.resize ( 4 );
    for ( int i=0;i<4;i++ )
    {
      pch=strtok ( NULL,"," );
      js.position[i]=boost::lexical_cast<double> ( pch );
    }
    js.header.seq=seq++;
    joint_state_pub.publish ( js );
  }
  close ( s );
  return;
}


bool get_joint_angles_srv ( bosch_arm_srvs::GetJointAngles::Request &req,
                            bosch_arm_srvs::GetJointAngles::Response &res )
{
  res.joint_angles=js.position;
  return true;
}

void trajCmdCallback ( const trajectory_msgs::JointTrajectory& msg )
{
//add a loop that executes on point at a time
  int hSock = socket ( AF_INET, SOCK_DGRAM, 0 );
  if ( hSock <=-1 )
    return;
  struct hostent *pServer = gethostbyname ( "localhost" );
  struct sockaddr_in addr;
  memset ( &addr, 0, sizeof ( addr ) );
  addr.sin_family = AF_INET;
  memcpy ( &addr.sin_addr.s_addr, pServer->h_addr, pServer->h_length );
  addr.sin_port = htons ( 10051 );

  char data[100];

    for ( unsigned int j=0;j<msg.points.size();j++ )
    {
      if(!ros::ok())
        return;
      sprintf ( data,"%%g90j1p%fj2p%fj3p%fj4p%f",
                msg.points[j].positions[0],
                msg.points[j].positions[1],
                msg.points[j].positions[2],
                msg.points[j].positions[3] );
      int n = sendto ( hSock, data, strlen ( data ), 0, ( sockaddr* ) &addr, sizeof ( addr ) );
      usleep ( 5000 );
    }
  

}
int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "wrapper" );
  ros::NodeHandle n;
//  ros::Publisher actuator_pub = n.advertise<openarms::ArmActuators>("arm_actuators_autopilot", 1);
  joint_state_pub =  n.advertise<sensor_msgs::JointState> ( "/joint_states",100 );
  boost::thread t2 = boost::thread::thread ( boost::bind ( &pubJointStates ) );
  ros::Subscriber traj_cmd_sub = n.subscribe ( "/traj_cmd", 3, trajCmdCallback );
  ros::ServiceServer service = n.advertiseService ( "get_joint_angles",
                                                    get_joint_angles_srv );
  sensor_msgs::JointState js;
  ros::Rate loop_rate ( 400 );
  while ( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
