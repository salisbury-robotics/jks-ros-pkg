#ifndef TRAJECTORY_CONTROLLER_H
#define TRAJECTORY_CONTROLLER_H

#include "ros/ros.h"
#include "bosch_arm.h"
#include <deque>
#include <string>
#include <bosch_arm_control/ArmState.h>
#include <bosch_arm_control/PointCmd.h>
#include <bosch_arm_control/Diagnostic.h>

class MoveRelAction
{
  
public:
  bool finished;
  bool calibration;
  bosch_arm_control::PointCmd cmd;
  double init_cmd_pos[4];
  double rel[4];
  double acc[4];
  double* f[4];
  double* count[4];
  int t_acc;
  int t_lin;
  int t_dec;
  int t_end;
  int t;
  std::string name;
  MoveRelAction(std::vector<double> relpos,int t)
  {
    for(int i=0;i<4;i++)
      rel[i]=relpos[i];
    //std::cout<<rel[3]<<"---"<<std::endl;
    t_end=t;    
    calibration=false;
  }
  
  void initialize(const double *pos)
  {
    finished=false;
    t=0;
    cmd.coordinates.resize(4);
    cmd.velocity.resize(4);
    cmd.acceleration.resize(4);
    cmd.header.frame_id="joint_space";
    for(int i=0;i<4;i++)
    {
      init_cmd_pos[i]=pos[i];
      cmd.coordinates[i]=pos[i];
      cmd.velocity[i]=0;
      cmd.acceleration[i]=0;
    }
    //compute trajectory
     //a simple trajectory:accelerate, linear, decelerate, still
    t_dec=t_end-10;
    t_acc=100;
    t_lin=t_dec-t_acc;
    for (int i=0;i<4;i++)
    {
      double v_max=rel[i]/(t_lin);
      acc[i]=v_max/t_acc;
    }
  }
  void update()
  {
    t++;
    //std::cout<<t<<std::endl;
    if(t>t_end)
    {
      finished=true;
      return;
    }
    
    for (int i=0;i<4;i++)
    {
      
      if (t<=t_acc)
      {
        cmd.velocity[i]+=acc[i];
        cmd.coordinates[i]+=cmd.velocity[i];
        cmd.acceleration[i]=acc[i];
      }
      else if (t<=t_lin)
      {
        cmd.velocity[i]=t_acc*acc[i];
        cmd.coordinates[i]+=cmd.velocity[i];
        cmd.acceleration[i]=0;
      }
      else if (t<=t_dec)
      {
        cmd.velocity[i]-=acc[i];
        cmd.coordinates[i]+=cmd.velocity[i];
        cmd.acceleration[i]=-acc[i];
      }
      else if(t<=t_end)
      {
        cmd.coordinates[i]=init_cmd_pos[i]+rel[i];
        cmd.velocity[i]=0;
        cmd.acceleration[i]=0;
      }
      
    }
    
  }
};
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