#include "action.h"

MoveRelAction::MoveRelAction(std::vector<double> relpos,int t)
{
  for (int i=0;i<4;i++)
    rel[i]=relpos[i];
  //std::cout<<rel[3]<<"---"<<std::endl;
  t_end=t;
  calibration=false;
  name="";
}

MoveRelAction::MoveRelAction(std::vector<double> relpos,TrajectoryController* ptr,int t,bool calib,const char* str)
{
  for (int i=0;i<4;i++)
    rel[i]=relpos[i];
  //std::cout<<rel[3]<<"---"<<std::endl;
  t_end=t;
  calibration=calib;
  name.assign(str);
  ctr=ptr;
}

void MoveRelAction::initialize(const double *pos)
{
  finished=false;
  t=0;
  cmd.coordinates.resize(4);
  cmd.velocity.resize(4);
  cmd.acceleration.resize(4);
  cmd.header.frame_id="joint_space";
  for (int i=0;i<4;i++)
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
  if (calibration)
  {
    for (int i=0;i<4;i++)
    {
      f[i]=new double[629];
      count[i]=new double[629];
      for (int j=0;j<629;j++)
      {
        f[i][j]=0;
        count[i][j]=0;
      }
    }
  }
}
void MoveRelAction::update()
{
  t++;
  //std::cout<<t<<std::endl;
  if (t>t_end)
  {
    finished=true;
    if (calibration)
    {
      for (int j=0;j<4;j++)
      {
        for (int i=0;i<629;i++)
        {
          if (count[j][i]==0)
            continue;
          f[j][i]=f[j][i]/count[j][i];
        }
      }
      ofstream out;
      string str=name;
      str.append(".txt");
      out.open(str.c_str());
      for (int i=0;i<629;i++)
      {
        for (int j=0;j<3;j++)
          out<<f[j][i]<<',';
        out<<f[3][i]<<endl;
      }
      out.close();
    }
    return;
  }

  for (int i=0;i<4;i++)
  {
    if (calibration)
    {
      int theta=round(ctr->q[i]/0.01)+314;
      count[i][theta]++;
      f[i][theta]+=ctr->f[i];
    }
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
    else if (t<=t_end)
    {
      cmd.coordinates[i]=init_cmd_pos[i]+rel[i];
      cmd.velocity[i]=0;
      cmd.acceleration[i]=0;
    }

  }

}
