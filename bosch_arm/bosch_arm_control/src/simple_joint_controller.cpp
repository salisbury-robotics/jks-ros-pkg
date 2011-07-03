#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "bosch_arm.h"
#include "simple_joint_controller.h"
#include <bosch_arm_control/ArmState.h>
#include <bosch_arm_control/PointCmd.h>
#include <bosch_arm_control/Diagnostic.h>
using namespace std;
  void SimpleJointController::singlePtCmdCallBack(const bosch_arm_control::PointCmd& msg)
  {
    for (int i=0;i<4;i++)
      qd[i]=msg.coordinates[i];
  }


  SimpleJointController::SimpleJointController(BoschArm *ptr)
  {
    rob=ptr;
    diagnostic_pub =  n.advertise<bosch_arm_control::Diagnostic> ("/diagnostics",100);
    sigle_point_cmd_sub = n.subscribe("/cartesian_point_cmd",3, &SimpleJointController::singlePtCmdCallBack, this);
    joint_state_pub =  n.advertise<bosch_arm_control::ArmState> ("/joint_states",100);
  }
  void SimpleJointController::start()
  {
    rob->initialize();
    rob->motor2JointPosition(rob->q,q);
    Kp[0]=20;
    Kp[1]=20;
    Kp[2]=5;
    Kp[3]=10;
    for (int i=0;i<4;i++)
    {
      //Kp[i]=10;
      Kv[i]= 0.1;
      v[i]=0;
      dq[i]=0;     
      f[i]=0;
      ql[i]=q[i];
      qd[i]=q[i];
    }
  }

  void SimpleJointController::update()
  {
    rob->update();
    //t_now=rob->time_now;
    clock_gettime(CLOCK_REALTIME, &ts);

    //save the last joint position
    for (int i=0;i<4;i++)
      ql[i]=q[i];
    rob->motor2JointPosition(rob->q,q);
    //PD control in cartesian space
    for (int i=0;i<4;i++)
    {

      dq[i]=qd[i]-q[i];
      v[i]= (q[i]-ql[i]) /rob->dt;
      f[i]=Kp[i]*dq[i]-Kv[i]*v[i];
    }

    //compute the torque at motor space
    //cout<<"transmission transpose:"<<endl;
    for (int i=0;i<4;i++)
    {
      torque[i]=0;
      for (int j=0;j<4;j++)
      {
        torque[i]+=rob->m2j[4*j+i]*f[j];
        //cout<<m2j[4*j+i]<<',';
      }
      //cout<<endl;
    }
    //cout<<"-------------------"<<endl;
    //cout<<"m    :";
//     for ( int i=0;i<4;i++ )
//       cout<<f[i]<<',';
//     cout<<endl;
    for (int i=0;i<4;i++)
      rob->torque[i]=torque[i];
//     for(int i=0;i<4;i++)
//       cout<<rob->q[i]<<',';
//     cout<<endl;
    //cout<<"m torque:";
//     for(int i=0;i<4;i++)
//       cout<<rob->torque[i]<<',';
//     cout<<endl;
    //cout<<"joint:";
//     for(int i=0;i<4;i++)
//       cout<<q[i]<<',';
//     cout<<endl;

//     cout<<endl<<"*********************************"<<endl;
    logging();

  }
  void SimpleJointController::logging()
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
    vector<double> cur_pos_act(q,q+4);
    js.header.stamp.sec=ts.tv_sec;
    js.header.stamp.nsec=ts.tv_nsec;
    js.position=cur_pos_act;
    joint_state_pub.publish(js);
  }
  void SimpleJointController::stop()
  {
    rob->close();
  }
