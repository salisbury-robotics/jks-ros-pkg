#ifndef MOVE_ACTION_H
#define MOVE_ACTION_H

#include <string>
#include <fstream>
#include <vector>
class MoveAction;
#include "trajectory_controller.h"
#include <bosch_arm_control/PointCmd.h>
using namespace std;

class MoveAction
{
public:
  bool finished;
  bool calibration;
  double* f[4];
  double* count[4];
  int t_acc;
  int t_lin;
  int t_dec;
  int t_end;
  int t;
  std::string name;
  TrajectoryController *ctr;
  bosch_arm_control::PointCmd cmd;
  
 virtual void initialize()=0;
 virtual void update()=0;
  
};

class MoveRelAction: public MoveAction
{
  
public:

  double init_cmd_pos[4];
  double rel[4];
  double acc[4];
  MoveRelAction(std::vector<double> relpos,TrajectoryController* ptr,int t);
  MoveRelAction(std::vector<double> relpos,TrajectoryController* ptr,int t,bool calib,const char* str);
  
  void initialize();
  void update();
};

class MoveAbsAction: public MoveAction
{
  
public:

  double init_cmd_pos[4];
  double des[4];
  double acc[4];
  MoveAbsAction(std::vector<double> relpos,TrajectoryController* ptr,int t);
  MoveAbsAction(std::vector<double> relpos,TrajectoryController* ptr,bool calib,const char* str);
  MoveAbsAction(const char* str){name.assign(str);}
  void initialize();
  void update();
  void loadTrace();
};

#endif