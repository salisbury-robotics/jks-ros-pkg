#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "bosch_arm.h"
#include <kdl/frames.hpp>
#include "trajectory_controller3.h"
#include <bosch_arm_control/ArmState.h>
#include <bosch_arm_control/PointCmd.h>
#include <bosch_arm_control/Diagnostic.h>
#include "cc2.h"
#include <Eigen/Core>
#include<Eigen/Array>
#include <Eigen/LU>
#include <Eigen/QR>

using namespace std;
void TrajectoryController::singlePtCmdCallBack(const bosch_arm_control::PointCmd& msg)
{
  MoveAbsAction* a=new MoveAbsAction(msg.coordinates,this,msg.duration);
  act_que.push_back(a);
}


TrajectoryController::TrajectoryController(BoschArm *ptr)
{
  rob=ptr;
  diagnostic_pub =  n.advertise<bosch_arm_control::Diagnostic> ("/diagnostics",100);
  sigle_point_cmd_sub = n.subscribe("/joint_point_cmd",3, &TrajectoryController::singlePtCmdCallBack, this);
  joint_state_pub =  n.advertise<bosch_arm_control::ArmState> ("/joint_states",100);
  js.position.resize(4);
  js.velocity.resize(4);
  js.effort.resize(4);
  Kp[0]=20;
  Kp[1]=20;
  Kp[2]=5;
  Kp[3]=10;
//    for(int i=0;i<4;i++)
//      Kp[i]=Kp[i]*0.02;
  Kv[0]=0.2;
  Kv[1]=0.1;
  Kv[2]=0.05;
  Kv[3]=0.1;
//   for(int i=0;i<4;i++)
//     Kv[i]=0;
  joint_limit_low[0]=-1.7;
  joint_limit_high[0]=2.1;
  joint_limit_low[1]=-0.6;
  joint_limit_high[1]=0.55;
  joint_limit_low[2]=-2.5;
  joint_limit_high[2]=2.3;
  joint_limit_low[3]=-2.3;
  joint_limit_high[3]=2.4;
  
}

//calculate the frames given the joint values.
void TrajectoryController::updateKinematics()
{
  frm_cur[0]=frm_home[0]; 
  frm0[0]=frm_home[0];
  for(int i=1;i<=4;i++)
  {
    frm_cur[i].p=frm_home[i].p;
    frm_cur[i].M=frm_home[i].M*Rotation::RotZ(q[i-1]);
    frm0[i]=frm0[i-1]*frm_cur[i];
  }
  frm_cur[5]=frm_home[5];
  frm0[5]=frm0[4]*frm_cur[5];
  frmn[4]=frm_cur[5];
  Vector z(0,0,1);
  
  //jacobian[3](joint 4) = z4*p54
  for(int i=3;i>=0;i--)
  {
    frmn[i]=frm_cur[i+1]*frmn[i+1];
    jacobian[i]=z*frmn[i+1].p;
    jacobian[i]=frm0[i+1].M*jacobian[i];
  }
}

void TrajectoryController::start()
{
  rob->initialize();
  //L0,L3 are not available until rob is initialized.
  frm_home[0]=Frame::Identity();
  Rotation r(0,-1,0,
             0,0,-1,
             1,0,0);
  frm_home[1].M=r;
  frm_home[1].p=Vector(0,0,rob->L0);
  frm_home[2].M=r;
  frm_home[3].M=r;
  frm_home[4].M=r;
  frm_home[4].p=Vector(-rob->L5,0,rob->L3);
  frm_home[5].p=Vector(rob->L4,0,0);
  
  rob->motor2JointPosition(rob->q,q);

  for (int i=0;i<4;i++)
  {
    v[i]=0;
    dq[i]=0;
    f[i]=0;
    ql[i]=q[i];
    qd[i]=q[i];
    vd[i]=v[i];
  }
  //state=CALIBRATION;
  state=LOADTRACE;
  ctr_mode=GRAVITY;
  if(state==CALIBRATION)
  {
    genCalibTraj2();  
    cur_act=act_que.front();
    cur_act->initialize();
    act_que.pop_front();
  }
  else if(state==LOADTRACE)
  {
    for(int i=0;i<4;i++)
      gc[i]=Vector(constants::gravc[i*2],constants::gravc[i*2+1],0);
    vector<double> des(4,0);
//     for(int i=0;i<4;i++)
//       des[i]=q[i];
//     des[1]=joint_limit_low[1];
//     des[3]=joint_limit_low[3]+0.1;
//     act_que.push_back(new MoveAbsAction(des,this));
//     des[1]=joint_limit_high[1];
    act_que.push_back(new MoveAbsAction(des,this));
    cur_act=act_que.front();
    cur_act->initialize();
    act_que.pop_front();
    
    //state=SERVO;
    
  }
}

void TrajectoryController::genCalibTraj2()
{
  vector<double> des(4,0);
//  int t=8000;  
  //move the the zero position.
  act_que.push_back(new MoveAbsAction(des,this));
  des[0]=-1.5;
  des[1]=-0.5;
  des[2]=-2.2;
  des[3]=-1.7;
  cali1_act=new MoveAbsAction(des,this);
  calib1=new Calibration("calib1");
  cali1_act->calib=calib1;
  act_que.push_back(cali1_act);
  des[0]=0.7;
  des[1]=0;
  des[2]=0;
  des[3]=0;
  act_que.push_back(new MoveAbsAction(des,this));
  des[0]=-0.8;
  des[1]=-0.5;
  des[2]=-2.2;
  des[3]=-1.7;
  cali2_act=new MoveAbsAction(des,this);
  calib2=new Calibration("calib2");
  cali2_act->calib=calib2;
  act_que.push_back(cali2_act);
  
}


void TrajectoryController::calcGravityCompensation()
{
  for(int j=0;j<calib1->len_b;j++)
    if(calib1->count[j]>0&&calib2->count[j]>0)
      {
        calib1->b[j]-=calib2->b[j];
        calib1->count[j]=1;
      }
    
  for(int i=0;i<8;i++)
  {
    for(int j=0;j<calib1->len_b;j++)
    {
      if(calib1->count[j]>0)
      {
        calib1->A[i][j]-=calib2->A[i][j];
      }
    }
  }
  
  Eigen::MatrixXf AA(8,8);
  for(int i=0;i<8;i++)
  {
    for(int j=0;j<8;j++)
    {
      double s=0;
      for(int k=0;k<calib1->len_b;k++)
        if(calib1->count[k]>0)
          s+=calib1->A[i][k]*calib1->A[j][k];
      AA(i,j)=s/calib1->len_b;
    }
  }
  //cout<<AA;
  Eigen::VectorXf Ab(8);
  for(int i=0;i<8;i++)
  {
    double s=0;
    for(int j=0;j<calib1->len_b;j++)
      if(calib1->count[j]>0)
        s+=calib1->A[i][j]*calib1->b[j];
    Ab(i)=-s/calib1->len_b;
  }
  Eigen::MatrixXf M=AA.inverse()*Ab;
  //cout<<M<<endl;
  for(int i=0;i<4;i++)
  {
    gc[i]=Vector(M(i*2,0),M(i*2+1,0),0);
  }
  for(int i=0;i<4;i++)
  {
    for(int j=0;j<3;j++)
      cout<<gc[i][j]<<',';
    cout<<endl;
  }
}

void TrajectoryController::update()
{
  rob->update();
  clock_gettime(CLOCK_REALTIME, &ts);
  //save the last joint position
  for (int i=0;i<4;i++)
    ql[i]=q[i];
  rob->motor2JointPosition(rob->q,q);
  rob->motor2JointVelocity(rob->v,v);

  if (state==CALIBRATION)
  {
    //update desired position
    if (cur_act->finished)
    {
       if(cur_act->calib==NULL)
         delete cur_act;
      
      if (act_que.size()==0)
      {
        calcGravityCompensation();
        state=SERVO;
        return;
      }
      cur_act=act_que.front();
      //cout<<cur_act->rel[3]<<endl;
      cur_act->initialize();
      act_que.pop_front();
    }
    cur_act->update();
    for (int i=0;i<4;i++)
    {
      qd[i]=cur_act->cmd.coordinates[i];
      vd[i]=cur_act->cmd.velocity[i];

    }
    //update torque command
    PDControl();
    
    logging();
  }
  else if (state==LOADTRACE)
  {
    if (cur_act->finished)
    {
       if(cur_act->calib==NULL)
         delete cur_act;
      
      if (act_que.size()==0)
      {
        //calcGravityCompensation();
        state=SERVO;
        return;
      }
   
      cur_act=act_que.front();
      //cout<<cur_act->rel[3]<<endl;
      cur_act->initialize();
      act_que.pop_front();
      
    }
    cur_act->update();
    for (int i=0;i<4;i++)
    {
      qd[i]=cur_act->cmd.coordinates[i];
      vd[i]=cur_act->cmd.velocity[i];

    }
    //update torque command
    PDWithGC();
    //PDControl();
    logging();
  }
  else if (state==SERVO)
  {
    //PDControl();
    if(ctr_mode==PDWITHGC)
      PDWithGC();
    else if(ctr_mode==PDCONTROL)
      PDControl();
    else if(ctr_mode==GRAVITY)
      gravity_compensation();
    else if(ctr_mode==NOCONTROL)
    {
      for(int i=0;i<4;i++)
        rob->torque[i]=0;
    }
    //floating();
    //gravity_compensation();
    logging();
  }
}

void TrajectoryController::gravity_compensation()
{

  updateKinematics();
  //T=sum(gcj) dot (g*Zi
  Vector g(0,0,-1);
  //double tq[4];
  for(int i=0;i<4;i++)
  {
    Vector Zi=frm0[i+1].M.UnitZ();
    f[i]=0;
    for(int j=i;j<4;j++)
    {
      Rotation R0j=frm0[j+1].M.Inverse();
      Vector tmp=R0j*(g*Zi);
      f[i]-=dot(gc[j],tmp);
    }
  }
  
  for (int i=0;i<4;i++)
  {
    torque[i]=0;
    for (int j=0;j<4;j++)
    {
      torque[i]+=rob->m2j[4*j+i]*f[j];

    }

  }
  for (int i=0;i<4;i++)
    rob->torque[i]=torque[i];
  //logging();
  
}

void TrajectoryController::PDWithGC()
{
  updateKinematics();
  for (int i=0;i<4;i++)
  {

    dq[i]=qd[i]-q[i];
    f[i]=Kp[i]*dq[i]-Kv[i]*(v[i]-vd[i]);
  }
  
  //T=sum(gcj) dot (g*Zi
  Vector g(0,0,-1);
  //double tq[4];
  for(int i=0;i<4;i++)
  {
    Vector Zi=frm0[i+1].M.UnitZ();
    for(int j=i;j<4;j++)
    {
      Rotation R0j=frm0[j+1].M.Inverse();
      Vector tmp=R0j*(g*Zi);
      f[i]-=dot(gc[j],tmp);
    }
  }
  for (int i=0;i<4;i++)
  {
    torque[i]=0;
    for (int j=0;j<4;j++)
    {
      torque[i]+=rob->m2j[4*j+i]*f[j];

    }

  }
  for (int i=0;i<4;i++)
    rob->torque[i]=torque[i];
  //logging();
}


void TrajectoryController::PDControl()
{
  for (int i=0;i<4;i++)
  {

    dq[i]=qd[i]-q[i];
    f[i]=Kp[i]*dq[i]-Kv[i]*(v[i]-vd[i]);
  }
  for (int i=0;i<4;i++)
  {
    torque[i]=0;
    for (int j=0;j<4;j++)
    {
      torque[i]+=rob->m2j[4*j+i]*f[j];

    }

  }
  for (int i=0;i<4;i++)
    rob->torque[i]=torque[i];
  //logging();
}


void TrajectoryController::logging()
{
  std::string log;
  std::stringstream out;
  for (int i=0;i<4;i++)
    out<<q[i]<<",";
  for (int i=0;i<4;i++)
    out<<qd[i]<<",";
  for (int i=0;i<4;i++)
    out<<v[i]<<",";
  for (int i=0;i<4;i++)
    out<<f[i]<<",";
  for (int i=0;i<4;i++)
    out<<(q[i]-qd[i])<<",";
  for (int i=0;i<4;i++)
    out<<rob->q[i]<<',';
  for (int i=0;i<4;i++)
    out<<rob->v[i]<<',';
  for (int i=0;i<4;i++)
    out<<rob->torque[i]<<',';
  out << ts.tv_sec << ','<< ts.tv_nsec;
  log = out.str();
  diag.data=log;
  diag.header.stamp.sec=ts.tv_sec;
  diag.header.stamp.nsec=ts.tv_nsec;
  diagnostic_pub.publish(diag);
  //vector<double> cur_pos_act(q,q+4);
  js.header.stamp.sec=ts.tv_sec;
  js.header.stamp.nsec=ts.tv_nsec;
  //js.position=cur_pos_act;

  for (int i=0;i<4;i++)
  {
    js.position[i]=q[i];
    js.velocity[i]=v[i];
    js.effort[i]=f[i];
  }
  joint_state_pub.publish(js);
}
void TrajectoryController::stop()
{
  rob->close();
}
