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
#include "bosch_arm_srvs/GetJointAngles.h"
#include "bosch_arm_srvs/SetJointAngles.h"
using namespace std;
int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "touch_stop" );
  ros::NodeHandle n;
  ros::ServiceClient client_get = n.serviceClient<bosch_arm_srvs::GetJointAngles> ( "get_joint_angles" );
  ros::ServiceClient client_set = n.serviceClient<bosch_arm_srvs::SetJointAngles> ( "set_joint_angles" );
  bosch_arm_srvs::GetJointAngles srv_get;
  bosch_arm_srvs::SetJointAngles srv_set;
  ros::Rate loop_rate ( 400 );
  double acc=boost::lexical_cast<double> ( argv[1] );
  double threshold=boost::lexical_cast<double> ( argv[2] );
  double adjust=boost::lexical_cast<double> ( argv[3] );
  double nacc=boost::lexical_cast<int> ( argv[4] );
  double q1err;
  client_get.call ( srv_get );
  srv_set.request.joint_angles=srv_get.response.joint_angles;
  srv_set.request.joint_angles[0]+=adjust;
  int nstep=1;

  while ( ros::ok() )
  {  
    client_set.call ( srv_set );
    loop_rate.sleep();
    client_get.call ( srv_get );
    q1err=srv_get.response.joint_angles[0]-srv_set.request.joint_angles[0];
    if(nstep<=nacc)
    {
      srv_set.request.joint_angles[0]+=nstep*acc;
      ++nstep;
    }else
      srv_set.request.joint_angles[0]+=nstep*acc;
    if ( q1err>threshold )
      break;
  }
  return 0;
}
