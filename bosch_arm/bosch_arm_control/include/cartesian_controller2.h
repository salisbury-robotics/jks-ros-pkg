#ifndef CARTESIAN_CONTROLLER_H
#define CARTESIAN_CONTROLLER_H

#include "ros/ros.h"
#include "bosch_arm.h"
#include <deque>
#include <bosch_arm_control/ArmState.h>
#include <bosch_arm_control/PointCmd.h>
#include <bosch_arm_control/Diagnostic.h>
class CartesianController;
#include "action_cart.h"

#include <kdl/frames.hpp>
#include <Eigen/Core>
#include<Eigen/Array>
#include <Eigen/LU>
#include <Eigen/QR>
using namespace KDL;


class CartesianController
{
private:
  double xl[3];//last tip position
  double dx[3];//tip position error
  //double torque[4];//motor torque
  double tq[4];
  double tm[4];
  
  double qv[4];
  double qd[4];
  double dq[4];

  BoschArm* rob;
  ros::NodeHandle n;
  ros::Publisher diagnostic_pub;
  ros::Publisher joint_state_pub;
  ros::Subscriber sigle_point_cmd_sub;
  bosch_arm_control::Diagnostic diag;
  bosch_arm_control::ArmState js;
  std::deque<MoveAbsAction*> act_que;
  MoveAbsAction* cur_act;

  MoveAbsAction* cali1_act;
  MoveAbsAction* cali2_act;
  MoveAbsAction* cali3_act;
  Calibration* calib1;
  Calibration* calib2;
  Calibration* calib3;
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
  Vector link[4];
  Eigen::Matrix4f nullspace;
  double Kc[4];
  double x[3];
  double q[4];
  double Kp[3];
  double Kv[3];
  double xd[3];
  //double qd_cali[4];
  double v[3];
  double vd[3];
  double f[3];
  timespec ts;
  enum {CALIBRATION, SERVO,LOADTRACE,SEMIAUTO};
  bool start_semi;
  bool end_semi;
  enum {PDCONTROL, PDWITHGC, GRAVITY, NOCONTROL};
  int state;
  int ctr_mode;
  CartesianController(BoschArm *ptr);
  void updateKinematics();
  void start();
  //void start2();
  void update();
  void PDControl();
  void PDWithGC();
  //void floating();
  void gravity_compensation();
  void calcLinkLength();
  void logging();
  void stop();
  //void genCalibTraj1();
  void genCalibTraj2();

};
#endif
