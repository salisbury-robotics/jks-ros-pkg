#include "action.h"

MoveRelAction::MoveRelAction(std::vector<double> relpos,TrajectoryController* ptr,int t)
{
  for (int i=0;i<4;i++)
    rel[i]=relpos[i];
  //std::cout<<rel[3]<<"---"<<std::endl;
  t_end=t;
  calibration=false;
  name="";
  ctr=ptr;
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


void MoveRelAction::initialize()
{
  finished=false;
  t=0;
  cmd.coordinates.resize(4);
  cmd.velocity.resize(4);
  cmd.acceleration.resize(4);
  cmd.header.frame_id="joint_space";
  double *pos=ctr->qd;
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

















MoveAbsAction::MoveAbsAction(std::vector<double> despos,TrajectoryController* ptr,int tt)
{
  for (int i=0;i<4;i++)
    des[i]=despos[i];
  //std::cout<<rel[3]<<"---"<<std::endl;
  t_end=tt;
  calibration=false;
  name="";
  ctr=ptr;
}

MoveAbsAction::MoveAbsAction(std::vector<double> despos,TrajectoryController* ptr,bool calib,const char* str)
{
  for (int i=0;i<4;i++)
    des[i]=despos[i];
  //std::cout<<rel[3]<<"---"<<std::endl;
  t_end=0;
  calibration=calib;
  name.assign(str);
  ctr=ptr;
}

void MoveAbsAction::initialize()
{
  finished=false;
  t=0;
  cmd.coordinates.resize(4);
  cmd.velocity.resize(4);
  cmd.acceleration.resize(4);
  cmd.header.frame_id="joint_space";
  double *pos=ctr->qd;
  double rel[4];
  for (int i=0;i<4;i++)
  {
    init_cmd_pos[i]=pos[i];
    rel[i]=des[i]-pos[i];
    cmd.coordinates[i]=pos[i];
    cmd.velocity[i]=0;
    cmd.acceleration[i]=0;
  }
  //compute trajectory
  //a simple trajectory:accelerate, linear, decelerate, still
  //if(calibration)
  //if(1)
  //{
    double relabs=0;
    int imax;
    for(int i=0;i<4;i++)
      if(fabs(rel[i])>relabs)
      {
        relabs=fabs(rel[i]);
        imax=i;
      }
    t_end=floor(relabs/M_PI*10*1000);
    if(calibration)
      t_end*=2;
  //}
  if(t_end<200)
  {
    cout<<"duration less than 200!\n"<<endl;
    t_end=200;
  }
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

void MoveAbsAction::loadTrace()
{
  string line;
  string str=name.append(".txt");
  ifstream myfile (str.c_str());
  for(int i=0;i<4;i++)
  {
    f[i]=new double[629];
    count[i]=new double[629];
  }
  if (myfile.is_open())
  {
    int j=0;
    cout<<"load "<<str<<endl;
    char buf[200];
    while ( myfile.good() )
    {
      getline (myfile,line);
      char* pch;
      sprintf(buf,line.c_str());
      pch=strtok(buf,",");
      int i=0;
      while(pch!=NULL)
      {
        f[i][j]=atof(pch);
        if(f[i][j]!=0)
          count[i][j]=1;
        else
          count[i][j]=0;
        //cout<<f[i][j]<<",";
        pch=strtok(NULL,",");
        i++;
      }
      //cout<<endl;
      j++;
    }
    myfile.close();
  }

  else cout << "Unable to open file"; 

}

void MoveAbsAction::update()
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
      cmd.coordinates[i]=des[i];
      cmd.velocity[i]=0;
      cmd.acceleration[i]=0;
    }

  }

}