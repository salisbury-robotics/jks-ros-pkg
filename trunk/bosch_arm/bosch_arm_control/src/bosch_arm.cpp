#include <math.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include "bosch_arm.h"
#include "cc2.h"
#include "daq.h"
#include "ros/ros.h"

using namespace std;
void BoschArm::initialize()
{
  //r1 = constants::r1;
  //r2 = constants::r2;
  L0 = constants::L0;
  L3=constants::L3;
  L4=constants::L4;
  L5=constants::L5;
  for (int i=0;i<16;i++)
    m2j[i] =constants::m2j[i];
  for (int i=0;i<16;i++)
    j2m[i] =constants::j2m[i];
  rad_per_count=constants::rad_per_count;
  
  for(int i=0;i<4;i++)
    qoff[i]=constants::q_off[i];
  setup626();
//   bool ishomed=false;
//   bool sawIndex[4];
//   for(int i=0;i<4;i++)
//     sawIndex[i]=false;
//   cout<<"Homing...."<<endl;
//   while(!ishomed)
//   {
//     bool flag=true;
//     for(int i=0;i<4;i++)
//       if(!sawIndex[i])
//       {
//         if(homed(i))
//         {
//           sawIndex[i]=true;
// //           home_offsets[i]=read_encoder(i) *rad_per_count;
// //           cout<<"motor "<<i<<" home offset:"<<home_offsets[i]<<endl;
//         }else
//           flag=false;
//       }
//       
//     ishomed=flag;
//     usleep(1000);
//   }
//   cout<<"Ready!"<<endl;
//   for(int i=0;i<4;i++)
//     home_offsets[i]=0;
//   q[0] = -1*(read_encoder(0) *rad_per_count - home_offsets[0]);
//   q[1] = -1*(read_encoder(1) *rad_per_count - home_offsets[1]);
//   q[2] = -1*(read_encoder(2) *rad_per_count - home_offsets[2]);
//   q[3] = read_encoder(3) *rad_per_count - home_offsets[3];
  
  time_now=S626_CounterReadLatch(constants::board0,constants::cntr_chan);
  time_last = time_now;
  time_now2= time_now;
  time_last2= time_now;
  
  home_offsets[0]=read_encoder(0) *rad_per_count;
  home_offsets[1]=read_encoder(1) *rad_per_count;
  home_offsets[2]=read_encoder(2) *rad_per_count;
  home_offsets[3]=read_encoder(3) *rad_per_count;
  //using a IIR filter: [b3,a3]=butter(4,0.05)
  //it caused some non-vanishing noise. it cuts off too much frequency.
//   filter_order=4;
//   double tmp_a[]={ 1.0000,   -3.5897,    4.8513,   -2.9241,    0.6630};
//   double tmp_b[]={  0.0312e-3,    0.1250e-3,    0.1874e-3,    0.1250e-3,    0.0312e-3};
  //using a IIR filter: [b3,a3]=butter(4,0.2)
  //this one generates buzzing
//   filter_order=4;
//   double tmp_b[]={ 0.0048,    0.0193,    0.0289,    0.0193,    0.0048};
//   double tmp_a[]={ 1.0000,   -2.3695,    2.3140,   -1.0547,    0.1874};
  //using a IIR filter: [b3,a3]=butter(2,0.2)
  //this one generates buzzing
//   filter_order=2;
//   double tmp_b[]={ 0.0675,    0.1349,    0.0675};
//   double tmp_a[]={ 1.0000,   -1.1430,    0.4128};
 // using a IIR filter: [b3,a3]=butter(2,0.05)
//   filter_order=2;
//   double tmp_b[]={ 0.0055,    0.0111,    0.0055};
//   double tmp_a[]={ 1.0000,   -1.7786,    0.8008};
  
  filter_order=1;
  double lambda=0.0;
  double tmp_b[]={ 1.0-lambda,0};
  double tmp_a[]={ 1.0,-lambda};
//    filter_order=10;
//    double tmp_b[]={ 0.0144,    0.0304,    0.0724,    0.1245,    0.1668,    0.1830,    0.1668,  0.1245,    0.0724,    0.0304,    0.0144};
//    double tmp_a[]={ 1.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,};
   a=new double[filter_order+1];
   b=new double[filter_order+1];
  for(int i=0;i<filter_order+1;i++)
  {
    a[i]=tmp_a[i];
    b[i]=tmp_b[i];
  }
  for (int i=0;i<4;i++)
  {
    q[i]=0;
    ql[i]=q[i];
    v[i]=0;
    t_lims[i]=1.0*constants::t_max;
    x_his[i]=new double[filter_order];
    y_his[i]=new double[filter_order];
    for(int j=0;j<filter_order;j++)
    {
      x_his[i][j]=0;
      y_his[i][j]=0;
    }
  }
  double vlim=constants::v_lim;
  v_lims[0]=vlim;
  v_lims[1]=2*vlim;
  v_lims[2]=2*vlim;
  v_lims[3]=2*vlim;
  zero_torques();
  t=0;
  debug_filter=false;
}

double BoschArm::convertToWallTime(uint16_t now,uint16_t last)
{
  uint16_t t = (now - last);
  return (double) t*constants::cnt2sec;
}
//get the hardware time
uint16_t BoschArm::getTime()
{
  return S626_CounterReadLatch(constants::board0,constants::cntr_chan);
}

void BoschArm::motor2JointPosition(const double* motors, double* joints)
{
  for (int i=0;i<4;i++)
  {
    joints[i]=0;
    for (int j=0;j<4;j++)
      joints[i]+=m2j[4*i+j]*motors[j];
    joints[i]+=qoff[i];
  }
}



void BoschArm::motor2JointVelocity(const double* motors, double* joints)
{
  for (int i=0;i<4;i++)
  {
    joints[i]=0;
    for (int j=0;j<4;j++)
      joints[i]+=m2j[4*i+j]*motors[j];
  }
}

void BoschArm::joint2TipPosition(const double* joints, double* tip)
{
  double q1=joints[0];
  double q2=joints[1];
  double q3=joints[2];
  double q4=joints[3];

  tip[0]= - L4* (sin(q4) * (cos(q1) *cos(q3) - sin(q1) *sin(q2) *sin(q3)) + cos(q2) *cos(q4) *sin(q1))
          - L5* (cos(q4) * (cos(q1) *cos(q3) - sin(q1) *sin(q2) *sin(q3)) - cos(q2) *sin(q1) *sin(q4))
          - L3*cos(q2) *sin(q1);
  tip[1]=   L5*(sin(q2)*sin(q4) - cos(q2)*cos(q4)*sin(q3))
            - L4*(cos(q4)*sin(q2) + cos(q2)*sin(q3)*sin(q4))
            - L3*sin(q2);
  tip[2]=   L0
            - L4*(sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - cos(q1)*cos(q2)*cos(q4))
            - L5*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) + cos(q1)*cos(q2)*sin(q4))
            + L3*cos(q1)*cos(q2);
}

void BoschArm::getJacobianJoint2Tip(const double* joints, double* jacobian)
{
  double q1=joints[0];
  double q2=joints[1];
  double q3=joints[2];
  double q4=joints[3];

  jacobian[0]=  L4*(sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - cos(q1)*cos(q2)*cos(q4))
                + L5*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) + cos(q1)*cos(q2)*sin(q4))
                - L3*cos(q1)*cos(q2);
  jacobian[1]=  L4*(cos(q4)*sin(q1)*sin(q2) + cos(q2)*sin(q1)*sin(q3)*sin(q4))
                - L5*(sin(q1)*sin(q2)*sin(q4) - cos(q2)*cos(q4)*sin(q1)*sin(q3))
                + L3*sin(q1)*sin(q2);
  jacobian[2]=  L4*sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))
                + L5*cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2));
  jacobian[3]=  L5*(sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) + cos(q2)*cos(q4)*sin(q1))
                - L4*(cos(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) - cos(q2)*sin(q1)*sin(q4));
  jacobian[4]=  0;
  jacobian[5]=  L5*(cos(q2)*sin(q4) + cos(q4)*sin(q2)*sin(q3))
                - L4*(cos(q2)*cos(q4) - sin(q2)*sin(q3)*sin(q4))
                - L3*cos(q2);
  jacobian[6]=- L5*cos(q2)*cos(q3)*cos(q4)
              - L4*cos(q2)*cos(q3)*sin(q4);
  jacobian[7]=  L4*(sin(q2)*sin(q4) - cos(q2)*cos(q4)*sin(q3))
                + L5*(cos(q4)*sin(q2) + cos(q2)*sin(q3)*sin(q4));
  jacobian[8]=- L4*(sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) + cos(q2)*cos(q4)*sin(q1))
              - L5*(cos(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) - cos(q2)*sin(q1)*sin(q4))
              - L3*cos(q2)*sin(q1);
  jacobian[9]=  L5*(cos(q1)*sin(q2)*sin(q4) - cos(q1)*cos(q2)*cos(q4)*sin(q3))
                - L4*(cos(q1)*cos(q4)*sin(q2) + cos(q1)*cos(q2)*sin(q3)*sin(q4))
                - L3*cos(q1)*sin(q2);
  jacobian[10]= L4*sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2))
                + L5*cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2));
  jacobian[11]= L5*(sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - cos(q1)*cos(q2)*cos(q4))
                - L4*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) + cos(q1)*cos(q2)*sin(q4));
}
//   void joint2MotorPosition(const double* joints, double* motors)
//   {
//     for (int i=0;i<4;i++)
//     {
//       motors[i]=0;
//       for (int j=0;j<4;j++)
//         motors[i]+=j2m[4*i+j]*joints[j];
//     }
//   }

void BoschArm::safe_exit(const char* msg)
{
  close();
  ROS_ERROR(msg);
  //pthread_kill(servo, SIGTERM);
  exit(1);

}
void BoschArm::truncate(double &x, double max)
{
  if (x>max)
    x=max;
  if (x<-max)
    x=-max;
}
void BoschArm::enforce_safety()
{
  for (int i=0;i<4;i++)
  {
    if (fabs(v[i]) >v_lims[i])
      safe_exit("overspeed");
    if (fabs(torque[i]) >constants::t_max)
      safe_exit("torque command over max achievable");
    //truncate(torque[i],t_lims[i]);
  }
//     double max[4];
//     int id=0;
//     double ratio=-1;
//     for(int i=0;i<4;i++)
//     {
//       if(ratio<torque[i]/t_lims[i])
//       {
//         ratio=torque[i]/t_lims[i];
//         id=i;
//       }
//     }
//     for(int i=0;i<4;i++)
//     {
//       truncate(torque[i],torque[i]/ratio);
//     }
}

/*
b2=fir1(4,0.4)

b2 =

    0.0101    0.2203    0.5391    0.2203    0.0101
*/

void BoschArm::update()
{
  time_now2=S626_CounterReadLatch(constants::board0,constants::cntr_chan);
  q[0] = -1*(read_encoder(0) *rad_per_count - home_offsets[0]);
  q[1] = -1*(read_encoder(1) *rad_per_count - home_offsets[1]);
  q[2] = -1*(read_encoder(2) *rad_per_count - home_offsets[2]);
  q[3] = read_encoder(3) *rad_per_count - home_offsets[3];
  dt=convertToWallTime(time_now2,time_last2);
  time_last2=time_now2;
  
  double tmp[4];
  
//   for(int i=0;i<4;i++)
//     qraw[i]=q[i];
  
//   for(int i=0;i<4;i++)
//   {
//     tmp[i]=b[0]*q[i];
//     for(int j=0;j<filter_order;j++)
//       tmp[i]+=b[j+1]*x_his[i][j];
//     for(int j=0;j<filter_order;j++)
//       tmp[i]-=a[j+1]*y_his[i][j];
//     for(int j=filter_order;j>0;j--)
//       x_his[i][j]=x_his[i][j-1];
//     x_his[i][0]=q[i];
//     for(int j=filter_order;j>0;j--)
//       y_his[i][j]=y_his[i][j-1];
//     y_his[i][0]=tmp[i];
//     //q[i]=tmp[i];
//     q[i]=y_his[i][0];    
//   }  
//   
//   for(int i=0;i<4;i++)
//     v[i]=(y_his[i][0]-y_his[i][1])/dt;
  
  //calculate raw v;
  
  for (int i=0;i<4;i++)
  {
    v[i]= (q[i]-ql[i]) /dt;
    ql[i]=q[i];
  }
  
//   if(debug_filter)
//   {
//   t++;
//   //test filter
//   for(int i=0;i<4;i++){
//   if(t<10000)
//     v[i]=1;
//   else
//     v[i]=0;
//   }
//   }
  //filter v;
  
  
//   for(int i=0;i<4;i++)
//   {
//     tmp[i]=b[0]*v[i];
//     for(int j=0;j<filter_order;j++)
//       tmp[i]+=b[j+1]*x_his[i][j];
//     for(int j=0;j<filter_order;j++)
//       tmp[i]-=a[j+1]*y_his[i][j];
//     for(int j=filter_order;j>0;j--)
//       x_his[i][j]=x_his[i][j-1];
//     x_his[i][0]=v[i];
//     for(int j=filter_order;j>0;j--)
//       y_his[i][j]=y_his[i][j-1];
//     y_his[i][0]=tmp[i];
//     //q[i]=tmp[i];
//     v[i]=y_his[i][0];    
//   }
  
  enforce_safety();
  write_torque(0,torque[0]);
  write_torque(1,torque[1]);
  write_torque(2,torque[2]);
  write_torque(3,torque[3]);
}
void BoschArm::close()
{
  zero_torques();
  S626_InterruptEnable(constants::board0, FALSE);
  S626_CloseBoard(constants::board0);
  cout << "626 closed out." << endl;
}
void BoschArm::wait()
{
  do
  {
    time_now=getTime();
    //printf("%d\t",time_now);
  }
  while ((uint16_t)(time_now - time_last) < constants::cycle_counts);
  //printf("***\n");
  //printf("%d\n",(uint16_t)(time_now-time_last));
  time_last = time_now;
}
