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

#include <kdl/frames.hpp>
using namespace KDL;

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
  std::deque<MoveAction*> act_que;
  MoveAction* cur_act;
  MoveAction* grav_act[4];
  MoveAction* fric_act[4];
  MoveAction* forward_act;
  MoveAction* backward_act;
  MoveAction* horiz_act;
  MoveAction* cali1_act;
  MoveAction* cali2_act;
  MoveAction* cali3_act;
  double joint_limit_low[4];
  double joint_limit_high[4];
  void singlePtCmdCallBack(const bosch_arm_control::PointCmd& msg);
  //int cycle_count;
public:
  //0 base frame, 1-4 joint fram, 5 tip frame
  Frame frm_home[6];
  //frm_cur[0] the tf from base to world. g is in world frame
  Frame frm_cur[6];
  Frame frm0[6];//the transform from i to world
  Frame frmn[5];//the transform from frame 5 to i
  Vector jacobian[4];
  Vector gc[4];//the gravity compensation vector.
  double q[4];
  double Kp[4];
  double Kv[4];
  double qd[4];
  //double qd_cali[4];
  double v[4];
  double vd[4];
  double f[4];
  timespec ts;
  enum {CALIBRATION, SERVO,LOADTRACE};
  int state;
  TrajectoryController(BoschArm *ptr);
  void updateKinematics();
  void start();
  void start2();
  void update();
  void PDControl();
  void floating();
  void gravity_compensation();
  void calcGravityCompensation();
  void logging();  
  void stop();
  void genCalibTraj1();
  void genCalibTraj2();
  
};
#endif