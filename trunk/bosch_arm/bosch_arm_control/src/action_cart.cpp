#include "action_cart.h"
#include <kdl/frames.hpp>
#include "cc2.h"
using namespace KDL;
using namespace std;

MoveAbsAction::MoveAbsAction(std::vector<double> despos,CartesianController* ptr,int tt)
{
  for (int i=0;i<3;i++)
    des[i]=despos[i];
  //std::cout<<rel[3]<<"---"<<std::endl;
  t_end=tt;
  //calibration=false;
  //name="";
  ctr=ptr;
  calib=NULL;
}

MoveAbsAction::MoveAbsAction(std::vector<double> despos,CartesianController* ptr)
{
  for (int i=0;i<3;i++)
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
  cmd.coordinates.resize(3);
  cmd.velocity.resize(3);
  cmd.acceleration.resize(3);
  cmd.header.frame_id="cartesian_space";
  double *pos=ctr->xd;
  for (int i=0;i<3;i++)
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
    for (int i=0;i<3;i++)
      if (fabs(rel[i])>relabs)
      {
        relabs=fabs(rel[i]);
        imax=i;
      }
    t_end=floor(relabs*20*constants::cycle_hz);
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
  for (int i=0;i<3;i++)
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

  for (int i=0;i<3;i++)
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

void MoveAbsAction::update2()
{
  t++;
  //std::cout<<t<<std::endl;
  
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

void Calibration::initialize(CartesianController* cptr,MoveAbsAction* mptr)
{
  //the parameter to be calibrated has size 8

//put the b into its bins. b has size 4
//the b in the same bin has the same left side coefficients.

//there are n non-empty bins, so there are n*4 equations.
//the coefficients can be pre-computed.

  //compute the number of bins.
  ctr=cptr;
  traj=mptr;
  bin=0;

  n_bin=3000;
  len_b=n_bin;
  b=new double[len_b];
  count=new int[len_b];
  cout<<"initialing"<<endl;
  for (int i=0;i<len_b;i++)
  {
    b[i]=0;
    count[i]=0;
  }
  
  for (int i=0;i<8;i++)
  {
    A[i]=new double[len_b];
  }
//   for(int i=0;i<8;i++)
//   {
//     q[i]=new double[len_b];
//   }
 
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

// void Calibration::update()
// {
// //   if(traj->t%20!=1)
// //     return;
//   //calculate coefficient
//   //r=sum_r_i
//   if(traj->t<1)
//     return;
//   b[bin]+=-0.08;
//   count[bin]++;
//   for (int i=0;i<4;i++)
//   {
//     Rotation Ri0=ctr->frm0[i+1].M;        
//     A[i*2][bin]+=Ri0(2,0);
//     A[i*2+1][bin]+=Ri0(2,1);
//   }
//   
//   if(traj->t%100==1&&traj->t>1)
//   {
//     for(int i=0;i<8;i++)
//       A[i][bin]/=count[bin];
//     b[bin]/=count[bin];
//     bin++;
//   }
// 
// }

void Calibration::update()
{
  if(bin>=len_b)
    return;
  if(traj->t%20!=1)
    return;
  //calculate coefficient
  //r=sum_r_i
  b[bin]=-0.27;
  count[bin]++;
  for (int i=0;i<4;i++)
  {
    Rotation Ri0=ctr->frm0[i+1].M;        
    A[i*2][bin]=Ri0(2,0);
    A[i*2+1][bin]=Ri0(2,1);
//    q[i][bin]=ctr->q[i];
  }
  bin++;
}

void Calibration::finish()
{
  

  ofstream out;
  string str=name;
  str.append("l.txt");
  out.open(str.c_str());
  for (int i=0;i<bin;i++)
  {
    for (int j=0;j<8;j++)
      out<<A[j][i]<<',';
//     
    out<<b[i]<<endl;
  }
  out.close();
  cout<<"finish "<<name<<endl;
  //calculate M (A'A)^-1 (A'b)
  
 
  
}

