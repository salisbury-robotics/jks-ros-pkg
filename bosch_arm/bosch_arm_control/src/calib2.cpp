/** This determines the joint 4 orientation by gravity
    It locks the other joints and puts joint 4 at different positions.
    
    accelerometer can also determine orientation change.
    However, its own pose is unknown. 
**/
#include "ros/ros.h"
#include "bosch_arm_srvs/GetJointAngles.h"
#include "bosch_arm_srvs/SetJointAngles.h"
#include "sensor_msgs/JointState.h"
#include <bosch_arm_control/PointCommand.h>
ros::Publisher joint_cmd_pub;
sensor_msgs::JointState js;
void jointStateCallBack(const sensor_msgs::JointState& msg)
{
  js=msg;
}
int main(int argc, char** argv)
{
  //move joint 4 at a constant speed
  //read error
  ros::init ( argc, argv, "touch_stop" );
  ros::NodeHandle n;
  //ros::ServiceClient client_get = n.serviceClient<bosch_arm_srvs::GetJointAngles> ( "get_joint_angles" );
  //ros::ServiceClient client_set = n.serviceClient<bosch_arm_srvs::SetJointAngles> ( "set_joint_angles" );
  joint_cmd_pub =  n.advertise<bosch_arm_control::PointCommand> ( "/single_point_cmd",100 );
  ros::Subscriber joint_state_sub = n.subscribe ( "/joint_states", 3, jointStateCallBack );
  double rate=boost::lexical_cast<double> ( argv[1] );
  double step=boost::lexical_cast<double> ( argv[2] );
  //bosch_arm_srvs::GetJointAngles srv_get;
  //bosch_arm_srvs::SetJointAngles srv_set;
  double q1err;
  //choose a near position for the first set.
  //client_get.call ( srv_get );
  //srv_set.request.joint_angles=srv_get.response.joint_angles;
  ros::Rate loop_rate ( rate );
  bosch_arm_control::PointCommand cmd;
  cmd.joint_angles.resize(4);
  cmd.header.frame_id="";
  while(js.position.size()==0)
    ros::spinOnce();
  for(int i=0;i<4;i++)
    cmd.joint_angles[i]=js.position[i];
  //the first one is lost
  joint_cmd_pub.publish(cmd);
  //should I set sequence number?
  while(ros::ok())
  {   
    //set a position
    joint_cmd_pub.publish(cmd);
    //wait for the new position to be stable
    loop_rate.sleep();
    ros::spinOnce();
    q1err=js.position[0]-cmd.joint_angles[0];
    cmd.joint_angles[0]+=step;
    
  }
  return 0;
}

