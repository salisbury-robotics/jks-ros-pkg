#ifndef ACTION_CART_H
#define ACTION_CART_H

#include <string>
#include <fstream>
#include <vector>
class MoveAction;
class MoveAbsAction;
class Calibration;
#include "cartesian_controller2.h"
#include <bosch_arm_control/PointCmd.h>
using namespace std;




class MoveAction
{
public:
  bool finished;
  //bool calibration;
  int t;
  //std::string name;
  CartesianController *ctr;
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
  MoveAbsAction(std::vector<double> pos,CartesianController* ptr,int t);
  MoveAbsAction(std::vector<double> despos,CartesianController* ptr);
  //MoveAbsAction(std::vector<double> relpos,TrajectoryController* ptr,bool calib,const char* str);
  //MoveAbsAction(const char* str){name.assign(str);}
  void initialize();
  void update();
    void update2();
  //void loadTrace();
};

class Calibration
{
public:
  //f are put into bins indexed by the joint-trajectory.
  
  int bin;
  int n_bin;
  int len_b;
  double* b;
  int* count;
  //double* q[4];
  double* A[8];
  double link[8];
  double bin_step[4];
  CartesianController* ctr;
  MoveAbsAction* traj;
  string name;
  //Calibration(TrajectoryController* cptr,MoveAction* mptr);
  Calibration(const char* str){name.assign(str);}
  void initialize(CartesianController* cptr,MoveAbsAction* mptr);
  void update();

  void finish();
  void loadTrace();
  int q2bin(double *q);
  void bin2q(int bin,double* q);
};



#endif