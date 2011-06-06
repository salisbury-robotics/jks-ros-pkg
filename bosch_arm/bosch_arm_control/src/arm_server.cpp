/**
published topic: JointState of the bosch arm at 100Hz
provided service: set joint target value 
**/

#include <ros/ros.h>
#include "bosch_arm_srvs/SetJointAngles.h"
#include "sensor_msgs/JointState.h"
#include "control.h"

bool set_joint_angles_srv(bosch_arm_srvs::SetJointAngles::Request &req,
                          bosch_arm_srvs::SetJointAngles::Response &res)
{
  set_Joint_Pos(req.joint_angles);
  return true;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_server");
  ros::NodeHandle n;
//  ros::Publisher actuator_pub = n.advertise<openarms::ArmActuators>("arm_actuators_autopilot", 1);
  ros::Publisher joint_state_pub =  n.advertise<sensor_msgs::JointState>("/joint_states",100);
  ros::ServiceServer service = n.advertiseService("set_joint_angles",
                                                  set_joint_angles_srv);
  sensor_msgs::JointState js;
  ros::Rate loop_rate(100);
  int seq=0;
  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
    //TODO:read the joint value
    js.name.resize(4);
    js.name[0]="joint1";
    js.name[1]="joint2";
    js.name[2]="joint3";
    js.name[3]="joint4";
    js.position=get_Joint_Pos();
    js.velocity=get_Joint_Vel();

    //js.effort.resize(4);
    js.header.stamp=ros::Time::now();
    js.header.seq=seq++;
    js.header.frame_id="";
    //publish JointState
    joint_state_pub.publish(js);
    
    
  }
  
  return 0;
}
