#ifndef MOVE_ACTION_H
#define MOVE_ACTION_H

#include <string>
#include <fstream>
#include <vector>
class MoveAction;
class MoveAbsAction;
class Calibration;
#include "trajectory_controller3.h"
#include <bosch_arm_control/PointCmd.h>
using namespace std;




class MoveAction
{
public:
  bool finished;
  //bool calibration;
  int t;
  //std::string name;
  TrajectoryController *ctr;
  bosch_arm_control::PointCmd cmd;
  
 virtual void initialize()=0;
 virtual void update()=0;
  
};

class MoveAbsAction: public MoveAction
{
  
public:
  int t_acc;
  int t_lin;
  int t_dec;
  int t_end;
  double init_cmd_pos[4];
  double des[4];
  double rel[4];
  double acc[4];
  Calibration *calib;
  MoveAbsAction(std::vector<double> pos,TrajectoryController* ptr,int t);
  MoveAbsAction(std::vector<double> despos,TrajectoryController* ptr);
  //MoveAbsAction(std::vector<double> relpos,TrajectoryController* ptr,bool calib,const char* str);
  //MoveAbsAction(const char* str){name.assign(str);}
  void initialize();
  void update();
  //void loadTrace();
};

class Calibration
{
public:
  //f are put into bins indexed by the joint-trajectory.
  int n_bin;
  int len_b;
  double* b;
  int* count;
  double* A[8];
  double bin_step[4];
  TrajectoryController* ctr;
  MoveAbsAction* traj;
  string name;
  //Calibration(TrajectoryController* cptr,MoveAction* mptr);
  Calibration(const char* str){name.assign(str);}
  void initialize(TrajectoryController* cptr,MoveAbsAction* mptr);
  void update();
  void finish();
  void loadTrace();
  int q2bin(double *q);
  void bin2q(int bin,double* q);
};



#endif