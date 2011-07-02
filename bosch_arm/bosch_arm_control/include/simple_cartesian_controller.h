#ifndef SIMPLE_CARTESIAN_CONTROLLER_H
#define SIMPLE_CARTESIAN_CONTROLLER_H

#include "ros/ros.h"
#include "bosch_arm.h"
#include <bosch_arm_control/TipState.h>
#include <bosch_arm_control/PointCmd.h>
#include <bosch_arm_control/Diagnostic.h>
class SimpleCartesianController
{
private:
  double xl[3];//last tip position
  double dx[3];//tip position error
  double q[4];//joint position
  double tj[4];//joint effort
  double torque[4];//motor torque
  BoschArm* rob;
  ros::NodeHandle n;
  ros::Publisher diagnostic_pub;
  ros::Publisher tip_state_pub;
  ros::Subscriber sigle_point_cmd_sub;
  bosch_arm_control::Diagnostic diag;
  bosch_arm_control::TipState js;
  void singlePtCmdCallBack(const bosch_arm_control::PointCmd& msg);
public:
  double x[3];
  double Kp[3];
  double Kv[3];
  double xd[3];
  double v[3];
  double f[3];
  timespec ts;

  SimpleCartesianController(BoschArm *ptr);
  void start();
  void update();  
  void logging();  
  void stop();
  
};
#endif