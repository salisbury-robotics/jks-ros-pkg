#ifndef MOVE_ACTION_H
#define MOVE_ACTION_H

#include <string>
#include <fstream>
#include <vector>
class MoveRelAction;
#include "trajectory_controller.h"
#include <bosch_arm_control/PointCmd.h>
using namespace std;



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
  TrajectoryController *ctr;
  MoveRelAction(std::vector<double> relpos,int t);
  MoveRelAction(std::vector<double> relpos,TrajectoryController* ptr,int t,bool calib,const char* str);
  
  void initialize(const double *pos);
  void update();
};

#endif