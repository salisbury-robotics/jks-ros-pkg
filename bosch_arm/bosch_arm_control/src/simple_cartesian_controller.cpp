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
#include "simple_cartesian_controller.h"
#include <bosch_arm_control/TipState.h>
#include <bosch_arm_control/PointCmd.h>
#include <bosch_arm_control/Diagnostic.h>
using namespace std;
  void SimpleCartesianController::singlePtCmdCallBack(const bosch_arm_control::PointCmd& msg)
  {
    for (int i=0;i<3;i++)
      xd[i]=msg.coordinates[i];
  }


  SimpleCartesianController::SimpleCartesianController(BoschArm *ptr)
  {
    rob=ptr;
    diagnostic_pub =  n.advertise<bosch_arm_control::Diagnostic> ("/diagnostics",100);
    sigle_point_cmd_sub = n.subscribe("/cartesian_point_cmd",3, &SimpleCartesianController::singlePtCmdCallBack, this);
    tip_state_pub =  n.advertise<bosch_arm_control::TipState> ("/tip_states",100);
  }
  void SimpleCartesianController::start()
  {
    rob->initialize();
    rob->motor2JointPosition(rob->q,q);
    rob->joint2TipPosition(q,x);
    for (int i=0;i<3;i++)
    {
      Kp[i]=10;
      Kv[i] = 0.1;
      v[i]=0;
      dx[i]=0;     
      f[i]=0;
      xl[i]=x[i];
      xd[i]=x[i];
    }
  }



  void SimpleCartesianController::update()
  {
    rob->update();
    //t_now=rob->time_now;
    clock_gettime(CLOCK_REALTIME, &ts);

    //save the last tip position
    for (int i=0;i<3;i++)
      xl[i]=x[i];
    rob->motor2JointPosition(rob->q,q);
    rob->joint2TipPosition(q,x);
    //PD control in cartesian space
    for (int i=0;i<3;i++)
    {

      dx[i]=xd[i]-x[i];
      v[i]= (x[i]-xl[i]) /rob->dt;
      f[i]=Kp[i]*dx[i]-Kv[i]*v[i];
    }

    //compute the torque at motor space
    double jacobian[12];
    rob->getJacobianJoint2Tip(q,jacobian);
    //cout<<"jacobian transpose:"<<endl;
    for (int i=0;i<4;i++)
    {
      tj[i]=0;
      for (int j=0;j<3;j++)
      {
        tj[i]+=jacobian[4*j+i]*f[j];
        //cout<<jacobian[4*j+i]<<',';
      }
      // cout<<endl;
    }
    //cout<<"-------------------"<<endl;
    //cout<<"transmission transpose:"<<endl;
    for (int i=0;i<4;i++)
    {
      torque[i]=0;
      for (int j=0;j<4;j++)
      {
        torque[i]+=rob->m2j[4*j+i]*tj[j];
        //cout<<m2j[4*j+i]<<',';
      }
      //cout<<endl;
    }
    //cout<<"-------------------"<<endl;
    //cout<<"m    :";
//     for ( int i=0;i<3;i++ )
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
    //cout<<"joint f:";
//     for(int i=0;i<4;i++)
//       cout<<tj[i]<<',';
//
//     cout<<endl<<"*********************************"<<endl;
    logging();

  }
  void SimpleCartesianController::logging()
  {
    std::string log;
    std::stringstream out;
    for (int i=0;i<3;i++)
      out<<x[i]<<",";
    for (int i=0;i<3;i++)
      out<<xd[i]<<",";
    for (int i=0;i<3;i++)
      out<<v[i]<<",";
    for (int i=0;i<3;i++)
      out<<f[i]<<",";
    for (int i=0;i<3;i++)
      out<<(x[i]-xd[i])<<",";
    for (int i=0;i<4;i++)
      out<<q[i]<<",";
    for (int i=0;i<4;i++)
      out<<tj[i]<<",";
    for (int i=0;i<4;i++)
      out<<rob->q[i]<<",";
    for (int i=0;i<4;i++)
      out<<rob->v[i]<<",";
    for (int i=0;i<4;i++)
      out<<rob->torque[i]<<",";
    out << ts.tv_sec << ','<< ts.tv_nsec;

    log = out.str();
    diag.data=log;
    diag.header.stamp.sec=ts.tv_sec;
    diag.header.stamp.nsec=ts.tv_nsec;
    diagnostic_pub.publish(diag);
    vector<double> cur_pos_act(x,x+3);
    js.header.stamp.sec=ts.tv_sec;
    js.header.stamp.nsec=ts.tv_nsec;
    js.position=cur_pos_act;
    tip_state_pub.publish(js);
  }
  void SimpleCartesianController::stop()
  {
    rob->close();
  }
