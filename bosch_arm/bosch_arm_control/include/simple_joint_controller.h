#ifndef SIMPLE_JOINT_CONTROLLER_H
#define SIMPLE_JOINT_CONTROLLER_H

#include "ros/ros.h"
#include "bosch_arm.h"
#include <bosch_arm_control/ArmState.h>
#include <bosch_arm_control/PointCmd.h>
#include <bosch_arm_control/Diagnostic.h>
class SimpleJointController
{
private:
  double ql[4];//last tip position
  double dq[4];//tip position error
  double torque[4];//motor torque
  BoschArm* rob;
  ros::NodeHandle n;
  ros::Publisher diagnostic_pub;
  ros::Publisher joint_state_pub;
  ros::Subscriber sigle_point_cmd_sub;
  bosch_arm_control::Diagnostic diag;
  bosch_arm_control::ArmState js;
  void singlePtCmdCallBack(const bosch_arm_control::PointCmd& msg);
public:
  double q[4];
  double Kp[4];
  double Kv[4];
  double qd[4];
  double v[4];
  double vd[4];
  double f[4];
  timespec ts;

  SimpleJointController(BoschArm *ptr);
  void start();
  void update();  
  void logging();  
  void stop();
  
};
#endif