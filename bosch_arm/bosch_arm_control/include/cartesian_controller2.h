#ifndef CARTESIAN_CONTROLLER_H
#define CARTESIAN_CONTROLLER_H

#include "ros/ros.h"
#include "bosch_arm.h"
#include <deque>
#include <bosch_arm_control/ArmState.h>
#include <bosch_arm_control/PointCmd.h>
#include <bosch_arm_control/Diagnostic.h>
#include <GMS120/GMS120_measurement.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>

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
  sensor_msgs::PointCloud pc;
  tf::TransformBroadcaster br;

  ros::Publisher diagnostic_pub;
  ros::Publisher joint_state_pub;
  ros::Publisher point_pub;
  ros::Subscriber sigle_point_cmd_sub;
  ros::Subscriber GMS_sub;
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
  void GMSCallBack(const GMS120::GMS120_measurement& msg);
  //int cycle_count;
public:
  int n_cur;
  double tzero[4];
  double xzero[3];
  double qzero[4];
  double mzero[4];
  void set_position_ref();
  void set_gc();
  void clear_ref();
  //a frame is a point that has position and orientation.
  //frame index convention for frm_home, frm0, frm_cur: -1 world, 0 base, 1-4 R-joints, 5 tip, 6 scanner.  
  //The home tf from frame i to frame i-1
  Frame frm_home[6];
  Vector scanner_p;
  //The current tf from frame i to frame i-1
  Frame frm_cur[6];
  //The current tf from frame i to frame world
  Frame frm0[7];
  //The current tf from frame tip to frame i
  Frame frmn[5];//the transform from frame 5 to i
  Vector jacobian[4];
  Vector gc[4];//the gravity compensation vector.

  //Vector link[7];
  double mu;
  Eigen::Matrix4f nullspace;
  double Kc[4];
  double x[3];
  double q[4];
  double s[3];
  double Kp[3];
  double Kv[3];
  double Ks[3];
  double xd[3];
  //double qd_cali[4];
  double v[3];
  double vd[3];
  double f[3];
  timespec ts;
  enum {CALIBRATION, SERVO,LOADTRACE,SEMIAUTO};
  bool start_semi;
  bool end_semi;
  enum {PDCONTROL, PDWITHGC, GRAVITY, NOCONTROL, PEAKFINDER, FULLSCAN};
  int state;
  int ctr_mode;
  double last_inductance;
  double inductance_sum_last;
  int cnta;
  int search_mode;
  int cycle;
  double radius;
  double center_x;
  double center_y;
  double dcx;
  double dcy;
  Eigen::Matrix3f AA;
  Eigen::Vector3f Ab;
  Eigen::MatrixXf M;
  double n_sum;
  int fd;
  CartesianController(BoschArm *ptr);
  void peak_finder();
  void full_scan();
  void start_fullscan();
  void start_peak_finder();
  void startPDWithGC();
  void updateKinematics();
  void stopTrajectory();
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
