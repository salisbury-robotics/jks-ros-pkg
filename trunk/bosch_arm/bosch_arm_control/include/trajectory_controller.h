#ifndef TRAJECTORY_CONTROLLER_H
#define TRAJECTORY_CONTROLLER_H

#include "ros/ros.h"
#include "bosch_arm.h"
#include <deque>
#include <bosch_arm_control/ArmState.h>
#include <bosch_arm_control/PointCmd.h>
#include <bosch_arm_control/Diagnostic.h>
class TrajectoryController;
#include "action.h"



class TrajectoryController
{
private:
  double ql[4];//last tip position
  double dq[4];//tip position error
  double torque[4];//motor torque
  //double* friction[4];
  //double* friction_r[4];
  //int* count[4];
  BoschArm* rob;
  ros::NodeHandle n;
  ros::Publisher diagnostic_pub;
  ros::Publisher joint_state_pub;
  ros::Subscriber sigle_point_cmd_sub;
  bosch_arm_control::Diagnostic diag;
  bosch_arm_control::ArmState js;
  std::deque<MoveRelAction*> act_que;
  MoveRelAction* cur_act;
  MoveRelAction* grav_act[4];
  MoveRelAction* fric_act[4];
  MoveRelAction* forward_act;
  MoveRelAction* backward_act;
  MoveRelAction* horiz_act;
  void singlePtCmdCallBack(const bosch_arm_control::PointCmd& msg);
  //int cycle_count;
public:
  double q[4];
  double Kp[4];
  double Kv[4];
  double qd[4];
  //double qd_cali[4];
  double v[4];
  double vd[4];
  double f[4];
  timespec ts;
  enum {CALIBRATION, SERVO};
  int state;
  TrajectoryController(BoschArm *ptr);
  void start();
  void start2();
  void update();
  void PDControl();
  void floating();
  void gravity_compensation();
  void logging();  
  void stop();
  
};
#endif