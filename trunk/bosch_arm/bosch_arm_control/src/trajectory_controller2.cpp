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
#include "trajectory_controller.h"
#include <bosch_arm_control/ArmState.h>
#include <bosch_arm_control/PointCmd.h>
#include <bosch_arm_control/Diagnostic.h>
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
//   for(int i=0;i<4;i++)
//     Kv[i]=Kp[i]*0.01;
  Kv[0]=0.2;
  Kv[1]=0.1;
  Kv[2]=0.05;
  Kv[3]=0.1;
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
  //updateKinematics();
  //cycle_count=0;
  for (int i=0;i<4;i++)
  {
    v[i]=0;
    dq[i]=0;
    f[i]=0;
    ql[i]=q[i];
    qd[i]=q[i];
    vd[i]=v[i];
  }
  state=LOADTRACE;
  if(state==CALIBRATION)
  {
    genCalibTraj1();  
    cur_act=act_que.front();
    cur_act->initialize();
    act_que.pop_front();
  }
  else if(state==LOADTRACE)
  {
    for(int i=0;i<4;i++)
    {
      std::stringstream str;
      str<<"grav"<<i;
      grav_act[i]=new MoveAbsAction(str.str().c_str());
      ((MoveAbsAction*)grav_act[i])->loadTrace();
      std::stringstream str2;
      str2<<"fric"<<i;
      fric_act[i]=new MoveAbsAction(str2.str().c_str());
      ((MoveAbsAction*)fric_act[i])->loadTrace();
    }
    calcGravityCompensation();
    state=SERVO;
  }
}

void TrajectoryController::genCalibTraj2()
{
  vector<double> des(4,0);
  int t=8000;  
  //move the the zero position.
  act_que.push_back(new MoveAbsAction(des,this,t));
  des[0]=-1.5;
  des[1]=-0.5;
  des[2]=-2.2;
  des[3]=-1.7;
  act_que.push_back(new MoveAbsAction(des,this,true,"cali1"));
  des[0]=0.7;
  des[1]=0;
  des[2]=0;
  des[3]=0;
  act_que.push_back(new MoveAbsAction(des,this,t));
  des[0]=-0.8;
  des[1]=-0.5;
  des[2]=-2.2;
  des[3]=-1.7;
  act_que.push_back(new MoveAbsAction(des,this,true,"cali2"));
  
}

void TrajectoryController::genCalibTraj1()
{
  vector<double> des(4,0);
  des[0]=0;
  des[1]=0;
  des[2]=0;
  des[3]=joint_limit_low[3];
  int t=8000;
  act_que.push_back(new MoveAbsAction(des,this,t));
  
  des[0]=0;
  des[1]=0;
  des[2]=0;
  des[3]=joint_limit_high[3];
  grav_act[3]=new MoveAbsAction(des,this,true,"grav3");
  act_que.push_back(grav_act[3]);  
  
  des[0]=-M_PI/2;
  des[1]=0;
  des[2]=-M_PI/2;
  des[3]=joint_limit_low[3];
  t=16000;
  act_que.push_back(new MoveAbsAction(des,this,t));
  
  des[0]=-M_PI/2;
  des[1]=0;
  des[2]=-M_PI/2;
  des[3]=joint_limit_high[3];
  fric_act[3]=new MoveAbsAction(des,this,true,"fric3");  
  act_que.push_back(fric_act[3]);  
  
  
  
  des[0]=-M_PI/2;
  des[1]=0;
  des[2]=-M_PI/2;
  des[3]=0;
  t=8000;
  act_que.push_back(new MoveAbsAction(des,this,t));
  
  des[0]=-M_PI/2;
  des[1]=0;
  des[2]=joint_limit_low[2];
  des[3]=0;
  t=4000;
  act_que.push_back(new MoveAbsAction(des,this,t));
  
  des[0]=-M_PI/2;
  des[1]=0;
  des[2]=joint_limit_high[2];
  des[3]=0;
  grav_act[2]=new MoveAbsAction(des,this,true,"grav2");  
  act_que.push_back(grav_act[2]);  
  
  
  
  des[0]=-M_PI/2;
  des[1]=joint_limit_low[1];
  des[2]=0;
  des[3]=M_PI/2;
  t=8000;
  act_que.push_back(new MoveAbsAction(des,this,t));
  
  des[0]=-M_PI/2;
  des[1]=joint_limit_high[1];
  des[2]=0;
  des[3]=M_PI/2;
  fric_act[1]=new MoveAbsAction(des,this,true,"fric1");  
  act_que.push_back(fric_act[1]);
  
  des[0]=0;
  des[1]=joint_limit_low[1];
  des[2]=0;
  des[3]=M_PI/2;
  t=8000;
  act_que.push_back(new MoveAbsAction(des,this,t));
  
  des[0]=0;
  des[1]=joint_limit_high[1];
  des[2]=0;
  des[3]=M_PI/2;
  grav_act[1]=new MoveAbsAction(des,this,true,"grav1");  
  act_que.push_back(grav_act[1]);
  
  des[0]=0;
  des[1]=0;
  des[2]=joint_limit_low[2];
  des[3]=0;
  t=8000;
  act_que.push_back(new MoveAbsAction(des,this,t));
  
  des[0]=0;
  des[1]=0;
  des[2]=joint_limit_high[2];  
  des[3]=0;
  fric_act[2]=new MoveAbsAction(des,this,true,"fric2");  
  act_que.push_back(fric_act[2]);  
  
  
  
  
  des[0]=joint_limit_low[0];
  des[1]=0;
  des[2]=0;
  des[3]=M_PI/2;
  t=8000;
  act_que.push_back(new MoveAbsAction(des,this,t));
  
  des[0]=joint_limit_high[0];
  des[1]=0;
  des[2]=0;
  des[3]=M_PI/2;
  t=floor(abs(des[0])/M_PI*20*1000);
  grav_act[0]=new MoveAbsAction(des,this,true,"grav0");  
  act_que.push_back(grav_act[0]);
  
  des[0]=joint_limit_low[0];
  des[1]=0;
  des[2]=0;
  des[3]=M_PI/2;
  t=floor(abs(des[0])/M_PI*20*1000);
  fric_act[0]=new MoveAbsAction(des,this,true,"fric0");  
  act_que.push_back(fric_act[0]);  
}


void TrajectoryController::calcGravityCompensation()
{
  //gc4
  int flag[629];
  double saf,saa;
  for(int i=0;i<629;i++)
  {
    flag[i]=0;
    if(grav_act[3]->count[3][i]>0 && fric_act[3]->count[3][i]>0)
      flag[i]=1;
  }
  saf=0;
  saa=0;
  for(int i=0;i<629;i++)
  {
    if(flag[i]>0)
    {
      //g=(0,0,1), Z4=(0,-1,0), g*Z4=(-1,0,0), M4=(-M4x*sin,0,M4x*cos)
      double a=sin(i*0.01-3.14);
      double f=-(grav_act[3]->f[3][i]-fric_act[3]->f[3][i]);
      saa+=a*a;
      saf+=a*f;
    }
  }
  gc[3]=Vector(saf/saa,0,0);
  
  //gc3
  for(int i=0;i<629;i++)
  {
    flag[i]=0;
    if(grav_act[2]->count[2][i]>0 && fric_act[2]->count[2][i]>0)
      flag[i]=1;
  }
  saf=0;
  saa=0;
  for(int i=0;i<629;i++)
  {
    if(flag[i]>0)
    {
      //g=(0,0,1), Z3=(1,0,0), M4=(M4x,0,0), M3=(0,M3x*sin,-M3x*cos), g*Z3=(0,-1,0)
      //note: here M3z only contributes to -M2y.
      double a=-sin(i*0.01-3.14);
      double f=-(grav_act[2]->f[2][i]-fric_act[2]->f[2][i]);
      saa+=a*a;
      saf+=a*f;
    }
  }
  gc[2]=Vector(saf/saa,0,0);
  //assert M3x<0
  
    //gc2 without M4
  for(int i=0;i<629;i++)
  {
    flag[i]=0;
    if(grav_act[1]->count[1][i]>0 && fric_act[1]->count[1][i]>0)
      flag[i]=1;
  }
  saf=0;
  saa=0;
  for(int i=0;i<629;i++)
  {
    if(flag[i]>0)
    {
      //g=(0,0,1), Z2=(1,0,0), M4=(-M4x,0,0),M3=(M3x,0,0),M2=(0,M2y*sin,-M2y*cos) 
      //g*Z2=(0,-1,0)
      double a=-sin(i*0.01-3.14);
      double f=-(grav_act[1]->f[1][i]-fric_act[1]->f[1][i]);
      saa+=a*a;
      saf+=a*f;
    }
  }
  gc[1]=Vector(0,saf/saa,0);
  
  
  gc[0]=Vector(0,0,0);
  for(int i=0;i<4;i++)
  {
    cout<<"M"<<i<<":\t";
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
      if(!cur_act->calibration)
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
  else if (state==SERVO)
  {
    //PDControl();
    //floating();
    gravity_compensation();
    //logging();
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
  logging();
  
}

void TrajectoryController::floating()
{
  for (int i=0;i<3;i++)
  {
    dq[i]=qd[i]-q[i];
    f[i]=Kp[i]*dq[i]-Kv[i]*(v[i]-vd[i]);
  }
  int theta=round(q[3]/0.01)+314;
  if(v[3]>0.1)
    f[3]=forward_act->f[3][theta];
  else if(v[3]<-0.1)
    f[3]=backward_act->f[3][theta];
  else
    f[3]=(forward_act->f[3][theta]+backward_act->f[3][theta])/2;
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
  logging();
  
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
  logging();
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
