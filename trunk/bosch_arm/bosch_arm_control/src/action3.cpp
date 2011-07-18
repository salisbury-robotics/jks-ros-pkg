#include "action3.h"
#include <kdl/frames.hpp>
#include "cc2.h"
using namespace KDL;
using namespace std;

MoveAbsAction::MoveAbsAction(std::vector<double> despos,TrajectoryController* ptr,int tt)
{
  for (int i=0;i<4;i++)
    des[i]=despos[i];
  //std::cout<<rel[3]<<"---"<<std::endl;
  t_end=tt;
  //calibration=false;
  //name="";
  ctr=ptr;
  calib=NULL;
}

MoveAbsAction::MoveAbsAction(std::vector<double> despos,TrajectoryController* ptr)
{
  for (int i=0;i<4;i++)
    des[i]=despos[i];
  //std::cout<<rel[3]<<"---"<<std::endl;
  t_end=-1;
  //calibration=false;
  //name="";
  ctr=ptr;
  calib=NULL;
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
  for (int i=0;i<4;i++)
  {
    init_cmd_pos[i]=pos[i];
    rel[i]=des[i]-pos[i];
    cmd.coordinates[i]=pos[i];
    cmd.velocity[i]=0;
    cmd.acceleration[i]=0;
  }
  if (t_end<0)
  {
    double relabs=0;
    int imax;
    for (int i=0;i<4;i++)
      if (fabs(rel[i])>relabs)
      {
        relabs=fabs(rel[i]);
        imax=i;
      }
    t_end=floor(relabs/M_PI*20*constants::cycle_hz);
    if (calib!=NULL)
      t_end*=2;
  }
  if (t_end<2*constants::cycle_hz/10)
  {
    cout<<"duration less than 200!\n"<<endl;
    t_end=2*constants::cycle_hz/10;
  }
  t_dec=t_end-constants::cycle_hz/100;
  t_acc=constants::cycle_hz/10;
  t_lin=t_dec-t_acc;
  for (int i=0;i<4;i++)
  {
    double v_max=rel[i]/(t_lin);
    acc[i]=v_max/t_acc;
  }
  if (calib!=NULL)
    calib->initialize(ctr,this);
}



void MoveAbsAction::update()
{
  t++;
  //std::cout<<t<<std::endl;
  if (t>t_end)
  {
    finished=true;
    if (calib!=NULL)
      calib->finish();
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
    else if (t<=t_end)
    {
      cmd.coordinates[i]=des[i];
      cmd.velocity[i]=0;
      cmd.acceleration[i]=0;
    }

  }
  if (calib!=NULL)
    calib->update();
}

int Calibration::q2bin(double *q)
{
  double dist=0;
  for (int i=0;i<4;i++)
    dist+=(q[i]-traj->init_cmd_pos[i])*(q[i]-traj->init_cmd_pos[i]);
  return round(sqrt(dist)/0.01)+10;
}

void Calibration::bin2q(int bin,double* q)
{
  for (int i=0;i<4;i++)
    q[i]=traj->init_cmd_pos[i]+bin_step[i]*(bin-10);
  return;
}

void Calibration::initialize(TrajectoryController* cptr,MoveAbsAction* mptr)
{
  //the parameter to be calibrated has size 8

//put the b into its bins. b has size 4
//the b in the same bin has the same left side coefficients.

//there are n non-empty bins, so there are n*4 equations.
//the coefficients can be pre-computed.

  //compute the number of bins.
  ctr=cptr;
  traj=mptr;
  double dist=0;
  for (int i=0;i<4;i++)
  {
    dist+=traj->rel[i]*traj->rel[i];
  }
  int n=round(sqrt(dist)/0.01);
  n_bin=n+20;
  len_b=n_bin*4;
  b=new double[len_b];
  count=new int[len_b];
  cout<<"initialing"<<endl;
  for (int i=0;i<len_b;i++)
  {
    b[i]=0;
    count[i]=0;
  }
  for (int i=0;i<4;i++)
    bin_step[i]=mptr->rel[i]/n;

  for (int i=0;i<8;i++)
  {
    A[i]=new double[len_b];
  }
  double q[4];
  Frame frm0[6];
  Frame frm_cur;
  Vector g(0,0,-1);
  Vector Zi;
  for (int j=0;j<n_bin;j++)
  {
    bin2q(j,q);
    //Ti=sum>i Mj dot (g*Zi)
    frm0[0]=cptr->frm_home[0];
    for (int i=1;i<=4;i++)
    {
      frm_cur.p=cptr->frm_home[i].p;
      frm_cur.M=cptr->frm_home[i].M*Rotation::RotZ(q[i-1]);
      frm0[i]=frm0[i-1]*frm_cur;
    }

    //each bin contains 4 equations for 4 joints
    //each eqation contains 8 coefficients for 4 joints.
    //first equation:
    //A[0-2]=R02*(g*Z1),A[3-5]=R03*(g*Z1), A[6-8]=R04*(g*Z1)
    //second
    //A[0-2]=0,...
    for (int i=0;i<4;i++)
    {
      Zi=frm0[i+1].M.UnitZ();
      int n_eq=j*4+i;
      for (int k=0;k<i;k++)
      {
        for (int c=0;c<2;c++)
          A[k*2+c][n_eq]=0;
      }
      for (int k=i;k<4;k++)
      {
        Rotation R0k=frm0[k+1].M.Inverse();
        Vector tmp=R0k*(g*Zi);
        for (int c=0;c<2;c++)
          A[k*2+c][n_eq]=tmp[c];
      }
    }

  }

}

void Calibration::loadTrace()
{
//   string line;
//   string str=name.append(".txt");
//   ifstream myfile(str.c_str());
//   for (int i=0;i<4;i++)
//   {
//     f[i]=new double[629];
//     count[i]=new double[629];
//   }
//   if (myfile.is_open())
//   {
//     int j=0;
//     cout<<"load "<<str<<endl;
//     char buf[200];
//     while (myfile.good())
//     {
//       getline(myfile,line);
//       char* pch;
//       sprintf(buf,line.c_str());
//       pch=strtok(buf,",");
//       int i=0;
//       while (pch!=NULL)
//       {
//         f[i][j]=atof(pch);
//         if (f[i][j]!=0)
//           count[i][j]=1;
//         else
//           count[i][j]=0;
//         //cout<<f[i][j]<<",";
//         pch=strtok(NULL,",");
//         i++;
//       }
//       //cout<<endl;
//       j++;
//     }
//     myfile.close();
//   }
//
//   else cout << "Unable to open file";

}

void Calibration::update()
{
  int bin=q2bin(ctr->q);
  for (int i=0;i<4;i++)
  {
    count[bin*4+i]++;
    b[bin*4+i]+=ctr->f[i];
  }


}

void Calibration::finish()
{
  for (int j=0;j<4;j++)
  {
    for (int i=0;i<n_bin;i++)
    {
      int id=i*4+j;
      if (count[id]==0)
        continue;
      b[id]=b[id]/count[id];
    }
  }
  ofstream out;
  string str=name;
  str.append(".txt");
  out.open(str.c_str());
  for (int i=0;i<len_b;i++)
  {
    for (int j=0;j<8;j++)
      out<<A[j][i]<<',';
    out<<b[i]<<endl;
  }
  out.close();
  //calculate M (A'A)^-1 (A'b)

}
