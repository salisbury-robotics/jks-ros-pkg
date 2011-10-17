/** The joint space and cartesian space controller has
  * the wobbling problem due to structural flexibility.
  * This controller avoid this problem by closing the
  * at motor space.
  */

#ifndef TRAJECTORY_CONTROLLER_H
#define TRAJECTORY_CONTROLLER_H

#include "ros/ros.h"
#include "bosch_arm.h"
#include "lti.h"
#include <deque>
#include <bosch_arm_control/ArmState.h>
#include <bosch_arm_control/PointCmd.h>
#include <bosch_arm_control/Diagnostic.h>
class TrajectoryController;
// class MoveAction;
// class MoveAbsAction;
// class Calibration;
//class LTISys;
#include <kdl/frames.hpp>
using namespace KDL;
using namespace std;


/*
class MotorModel
{
public:
  LTISys* out_flt;
  LTISys* in_flt;
  
  double m;
  double md;
  double dm;
  double Kmp;
  double mv;
  double mvd;
  double dmv;
  double Kmv;
  double mt;
  double mtmax;
  double w_amp;
  double w_prd;
  double mzero;
  double fpara[6];
  double tzero;
};*/

class TrajectoryController
{
private:

  BoschArm* rob;
  ros::NodeHandle n;
  ros::Publisher diagnostic_pub;
  ros::Publisher joint_state_pub;
  ros::Subscriber sigle_point_cmd_sub;
  bosch_arm_control::Diagnostic diag;
  bosch_arm_control::ArmState js;
//   std::deque<MoveAbsAction*> act_que;
//   MoveAbsAction* cur_act;
//   MoveAbsAction* cali1_act;
//   MoveAbsAction* cali2_act;
//TODO Calibrator that takes a multi-waypoint trajectory.
//   Calibration* calib1;
//   Calibration* calib 2;
  
  double joint_limit_low[4];
  double joint_limit_high[4];
  void singlePtCmdCallBack(const bosch_arm_control::PointCmd& msg);
  //int cycle_count;
public:
  //MotorModel motor[4];
  LTISys* filter[4];
  LTISys* flt_delay[4];
  bool close_loop;
  int filter_order;
  double mt[4];//motor torque
  double md[4];
  double mvd[4];
  double m[4];
  double mv[4];
  double dm[4];
  double dmv[4];
  double Kmv[4];
  double Kmp[4];
  double mtmax[4];
  double mvstep[4];
  int mvtime[4];
  bool do_mvstep;
  bool do_msin;
  int mtvstep;
  void mstep(int joint, double step);
  void do_mVStep();
  void do_motorSine();
  double mzero[4];
  
  double *fpara[4];
  void updateFilter(int mid);

  double qt[4];//motor torque
  double qd[4];
  double qvd[4];
  double q[4];
  double qv[4];
  double dq[4];
  double dqv[4];
  double Kqv[4];
  double Kqp[4];
  double qtmax[4];
  double qvstep[4];
  int qvtime[4];
  bool do_qvstep;
  bool do_qsin;
  int qtvstep;
  void qstep(int joint, double step);
  void do_qVStep();
  void do_qSin();
  double qzero[4];
  
  void do_Chirp();
  void do_ChirpSine();
  void do_stretch();

  bool do_chirp;
  double chirp_step;
  double tzero[4];
  //0 base frame, 1-4 joint fram, 5 tip frame
  Frame frm_home[6];
  //frm_cur[0] the tf from base to world. g is in world frame
  Frame frm_cur[6];
  Frame frm0[6];//the transform from i to world
  Frame frmn[5];//the transform from frame 5 to i
  Vector jacobian[4];
  Vector gc[4];//the gravity compensation vector.

  int jid;

  timespec ts;
  //int tvstep;
  enum {CALIBRATION, SERVO,LOADTRACE};
  enum {QD_KQ_M2JT, QD_KQ_GC_M2JT, GC_M2JT, NOCONTROL, QD_J2M_KM,MD_KM,SYSID};
  int state;
  int ctr_mode;
  TrajectoryController(BoschArm *ptr);
  void updateKinematics();
  void start();
  //void start2();
  void update();
  void set_position_ref();
  void set_gc();
  void clear_ref();
  void qd_Kq_gc_m2jT();
  void md_Km();
  void md_Km_flt();
  void md_VarKm();
  void md_Km_VarKm_Comp();
  double max_torque;
  void qd_j2m_Km();
  void qd_Kq_m2jT();
  void gc_m2jT();

  //void calcGravityCompensation();
  void logging();


  void stop();
  //void genCalibTraj1();
  //void genCalibTraj2();

};

#endif
