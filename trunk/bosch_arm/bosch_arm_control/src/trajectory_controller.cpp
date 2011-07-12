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
#include "trajectory_controller.h"
#include <bosch_arm_control/ArmState.h>
#include <bosch_arm_control/PointCmd.h>
#include <bosch_arm_control/Diagnostic.h>
using namespace std;
void TrajectoryController::singlePtCmdCallBack(const bosch_arm_control::PointCmd& msg)
{
  MoveRelAction* a=new MoveRelAction(msg.coordinates,msg.duration);
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
  Kv[0]=0.1;
  Kv[1]=0.1;
  Kv[2]=0.1;
  Kv[3]=0.1;
}
void TrajectoryController::start2()
{
  rob->initialize();
  rob->motor2JointPosition(rob->q,q);

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

  state=CALIBRATION;
  //state=SERVO;
  vector<double> rel(4,0);
  //from -3.14 to 3.14
  //friction[3]=new double[629];
  //count[3]=new int[629];
  //memset(friction,sizeof(friction),0);
  //memset(count,sizeof(count),0);
  
  
  rel[0]=0-q[0];
  rel[1]=0-q[1];
  rel[2]=0-q[2];
  rel[3]=-2.31-q[3];
  int t=8000;
  act_que.push_back(new MoveRelAction(rel,t));
  
  rel[0]=0;
  rel[1]=0;
  rel[2]=0;
  rel[3]=5.0;
  t=floor(abs(rel[3])/M_PI*20*1000);
  forward_act=new MoveRelAction(rel,t);  
  forward_act->calibration=true;
  forward_act->name=string("forward");
  forward_act->f[3]=new double[629];
  forward_act->count[3]=new double[629];
  for(int i=0;i<629;i++)
  {
    forward_act->f[3][i]=0;
    forward_act->count[3][i]=0;
  }
  act_que.push_back(forward_act);  
  
  rel[3]=-5.0;
  backward_act=new MoveRelAction(rel,t);  
  backward_act->calibration=true;
  backward_act->name=string("backward");
  backward_act->f[3]=new double[629];
  backward_act->count[3]=new double[629];
  for(int i=0;i<629;i++)
  {
    backward_act->f[3][i]=0;
    backward_act->count[3][i]=0;
  }
  act_que.push_back(backward_act);  
  
  
  cur_act=act_que.front();
  cur_act->initialize(qd);
  act_que.pop_front();;
}
void TrajectoryController::start()
{
  rob->initialize();
  rob->motor2JointPosition(rob->q,q);

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

  state=CALIBRATION;
  //state=SERVO;
  vector<double> rel(4,0);
  
  rel[0]=0-q[0];
  rel[1]=0-q[1];
  rel[2]=0-q[2];
  rel[3]=-2.31-q[3];
  int t=8000;
  act_que.push_back(new MoveRelAction(rel,t));
  
  rel[0]=0;
  rel[1]=0;
  rel[2]=0;
  rel[3]=4.7;
  t=floor(abs(rel[3])/M_PI*20*1000);
  forward_act=new MoveRelAction(rel,t);  
  forward_act->calibration=true;
  forward_act->name=string("forward");
  forward_act->f[3]=new double[629];
  forward_act->count[3]=new double[629];
  for(int i=0;i<629;i++)
  {
    forward_act->f[3][i]=0;
    forward_act->count[3][i]=0;
  }
  act_que.push_back(forward_act);  
  
  rel[0]=-M_PI/2;
  rel[1]=0;
  rel[2]=-M_PI/2;
  rel[3]=-4.7;
  t=16000;
  act_que.push_back(new MoveRelAction(rel,t));
  
  rel[0]=0;
  rel[1]=0;
  rel[2]=0;
  rel[3]=4.7;
  t=floor(abs(rel[3])/M_PI*20*1000);
  horiz_act=new MoveRelAction(rel,t);  
  horiz_act->calibration=true;
  horiz_act->name=string("horiz");
  horiz_act->f[3]=new double[629];
  horiz_act->count[3]=new double[629];
  for(int i=0;i<629;i++)
  {
    horiz_act->f[3][i]=0;
    horiz_act->count[3][i]=0;
  }
  act_que.push_back(horiz_act);  
  
  
  rel[0]=M_PI/2;
  rel[1]=0-q[1];
  rel[2]=M_PI/2;
  rel[3]=-4.7-q[3];
  t=8000;
  act_que.push_back(new MoveRelAction(rel,t));
  
  
  cur_act=act_que.front();
  cur_act->initialize(qd);
  act_que.pop_front();;
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
      if(cur_act->calibration)
      {
        for(int i=0;i<629;i++)
        {
          if(cur_act->count[3][i]==0)
            continue;
          cur_act->f[3][i]=cur_act->f[3][i]/cur_act->count[3][i];
        }
        ofstream out;
        string name=cur_act->name;
        name.append(".txt");
        out.open(name.c_str());
        for(int i=0;i<629;i++)
          out<<cur_act->f[3][i]<<endl;
        out.close();
      }
      else      
        delete cur_act;
      
      if (act_que.size()==0)
      {
        state=SERVO;
        return;
      }
      cur_act=act_que.front();
      //cout<<cur_act->rel[3]<<endl;
      cur_act->initialize(qd);
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
    if(cur_act->calibration)
    {
      int theta=round(q[3]/0.01)+314;
      cur_act->count[3][theta]++;
      cur_act->f[3][theta]+=f[3];
    }
    logging();
  }
  else if (state==SERVO)
  {
    //PDControl();
    //floating();
    gravity_compensation();
    logging();
  }
}

void TrajectoryController::gravity_compensation()
{
  for (int i=0;i<3;i++)
  {
    dq[i]=qd[i]-q[i];
    f[i]=Kp[i]*dq[i]-Kv[i]*(v[i]-vd[i]);
  }
  int theta=round(q[3]/0.01)+314;
  f[3]=forward_act->f[3][theta]-horiz_act->f[3][theta];
 
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
