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
#include "motor_controller2.h"
#include <bosch_arm_control/ArmState.h>
#include <bosch_arm_control/PointCmd.h>
#include <bosch_arm_control/Diagnostic.h>
#include "cc2.h"
#include <Eigen/Core>
#include<Eigen/Array>
#include <Eigen/LU>
#include <Eigen/QR>

/** This is an upgrade for motor_controller, LTISys is seperated as a standalone 
 *  library, calibration tools are removed. It tunes 4 motors all
 *  together.
**/

using namespace std;
void TrajectoryController::singlePtCmdCallBack(const bosch_arm_control::PointCmd& msg)
{
//   MoveAbsAction* a=new MoveAbsAction(msg.coordinates,this,msg.duration);
//   act_que.push_back(a);
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
  Kqp[0]=20;
  Kqp[1]=20;
  Kqp[2]=10;
  Kqp[3]=20;

  Kqv[0]=0.01;
  Kqv[1]=0.01;
  Kqv[2]=0.01;
  Kqv[3]=0.01;

  joint_limit_low[0]=-1.7;
  joint_limit_high[0]=2.1;
  joint_limit_low[1]=-0.6;
  joint_limit_high[1]=0.55;
  joint_limit_low[2]=-2.5;
  joint_limit_high[2]=2.3;
  joint_limit_low[3]=-2.3;
  joint_limit_high[3]=2.4;

  qtmax[0]=constants::t_max/constants::r1;
  qtmax[1]=constants::t_max/constants::r2;
  qtmax[2]=constants::t_max/constants::r2;
  qtmax[3]=constants::t_max/constants::r2;

  Kmp[0]=0.8;
  Kmp[1]=0.8;
  Kmp[2]=0.8;
  Kmp[3]=0.8;

  Kmv[0]=0.006;
  Kmv[1]=0.006;
  Kmv[2]=0.006;
  Kmv[3]=0.006;

  mtmax[0]=constants::t_max;
  mtmax[1]=constants::t_max;
  mtmax[2]=constants::t_max;
  mtmax[3]=constants::t_max;

  qvtime[0]=3000;
  qvtime[1]=1500;
  qvtime[2]=800;
  qvtime[3]=800;

  mvtime[0]=3000;
  mvtime[1]=1500;
  mvtime[2]=800;
  mvtime[3]=800;

  qvstep[0]=0.01;
  qvstep[1]=0.01;
  qvstep[2]=0.01;
  qvstep[3]=0.01;

  mvstep[0]=0.01;
  mvstep[1]=0.01;
  mvstep[2]=0.01;
  mvstep[3]=0.01;

  chirp_step=1;

  do_qvstep=false;
  do_mvstep=false;
  do_chirp=false;
  do_msin=false;
  do_qsin=false;
  close_loop=false;

  qtvstep=0;
  mtvstep=0;
  max_torque=0.05;
  clear_ref();

  for(int i=0;i<4;i++)
  {
    filter[i]=new LTISys(1);
    flt_delay[i]=new LTISys(1);
    fpara[i]=new double[6];
    fpara[i][0]=0;
    fpara[i][1]=0.25;
    fpara[i][2]=0;
    fpara[i][3]=0.25;
    fpara[i][4]=0;
    fpara[i][5]=0.3;
    fpara[i][6]=3;
    updateFilter(i);
    mvtime[i]=50;
  }
  
  
  fpara[0][0]=25;
  fpara[0][1]=0.25;
  fpara[0][2]=62;
  fpara[0][3]=0.25;
  fpara[0][4]=50;
  fpara[0][5]=0.3;
  fpara[0][6]=2.3;
  Kmp[0]=6;
  updateFilter(0);
  
  fpara[1][0]=16;
  fpara[1][1]=0.25;
  fpara[1][2]=34;
  fpara[1][3]=0.25;
  fpara[1][4]=40;
  fpara[1][5]=0.3;
  fpara[1][6]=2.3;
  Kmp[1]=6;
  updateFilter(1);
  
  fpara[2][0]=11;
  fpara[2][1]=0.2;
  fpara[2][2]=23;
  fpara[2][3]=0.25;
  fpara[2][4]=30;
  fpara[2][5]=0.3;
  fpara[2][6]=2.3;
  Kmp[2]=6;
  updateFilter(2);
  
  fpara[3][0]=25;
  fpara[3][1]=0.2;
  fpara[3][2]=55;
  fpara[3][3]=0.2;
  fpara[3][4]=50;
  fpara[3][5]=0.2;
  fpara[3][6]=2.3;
  Kmp[3]=6;
  
  
  updateFilter(3);



}

void TrajectoryController::updateFilter(int mid)
{
  double ts=1.0/constants::cycle_hz;
  //lead-lag compensation: boost up phase margin at cutoff frequency
  LTISys lead(1);
  LTISys lag(1);
  //LTISys leadlag(1);
  if(fpara[mid][4]>0)
  {
  double wc=fpara[mid][4]*2*M_PI;
  
  double alpha=1.0/fpara[mid][5];
  double wleadl=wc/alpha;
  double wleadh=wc*alpha;
  
  double wlagh=wc/10;
  double wlagl=wlagh/alpha;

  double sleadb[]={1,1/wleadl};
  double sleada[]={1,1/wleadh};
  double slagb[]={1,1/wlagh};
  double slaga[]={1,1/wlagl};
  double glag= sqrt((wc/wlagh*wc/wlagh+1)/(wc/wlagl*wc/wlagl+1));
  double glead=sqrt((wc/wleadl*wc/wleadl+1)/(wc/wleadh*wc/wleadh+1));
  lead=LTISys(sleadb,1,sleada,1,1);
  lag=LTISys(slagb,1,slaga,1,1/glag/glead);
  //leadlag=lead*lag;
  }
  //resonance compensation: supress leakage of high frequency noise
  LTISys reso(1);
  LTISys reso1(1);
  LTISys reso2(1);
  if(fpara[mid][0]>0&&fpara[mid][2]>0)
  {
  double w1=fpara[mid][0]*2*M_PI;
  double zeta1=fpara[mid][1];
  double w2=fpara[mid][2]*2*M_PI;
  double zeta2=fpara[mid][3];
  double sa1[]={1,2*zeta1/w1,1/w1/w1};
  double sb1[]={1,2*zeta2/w2,1/w2/w2};
  double ss[]={1};
  reso=LTISys(sb1,2,sa1,2);
  //reso1=LTISys(sb1,2,ss,0);
  //reso2=LTISys(ss,0,sa1,2);
  }
  LTISys overall(reso*lag);  
  //LTISys overall(reso2*lag);
  
  LTISys *tmp=filter[mid];  
  filter[mid]=new LTISys(overall.tustin(ts));  
  if(tmp->order_b>0)
    filter[mid]->initialize(tmp->x[0]);
  delete tmp;
  
  //delay compensation for input.
  LTISys delay(1);
  if(fpara[mid][6]>0)
  {
  double td=fpara[mid][6]*1e-3;
  double sdelayb[]={1,td};
  double sdelaya[]={1,td/10};
  delay=LTISys(sdelayb,1,sdelaya,1);
  }
  tmp=flt_delay[mid];
  LTISys overall2(delay*lead);
  flt_delay[mid]=new LTISys(overall2.tustin(ts));
  if(tmp->order_b>0)
    flt_delay[mid]->initialize(tmp->x[0]);
//  cout<<*flt_delay;
  delete tmp;
  
}

// void TrajectoryController::updateFilter(int mid)
// {
//   double ts=1.0/constants::cycle_hz;
//   //lead-lag compensation: boost up phase margin at cutoff frequency
//   LTISys leadlag(1);
//   if(fpara[mid][4]>0)
//   {
//   double wc=fpara[mid][4]*2*M_PI;
//   
//   double alpha=1.0/fpara[mid][5];
//   double wleadl=wc/alpha;
//   double wleadh=wc*alpha;
//   
//   double wlagh=wc/10;
//   double wlagl=wlagh/alpha;
// 
//   double sleadb[]={1,1/wleadl};
//   double sleada[]={1,1/wleadh};
//   double slagb[]={1,1/wlagh};
//   double slaga[]={1,1/wlagl};
//   double glag= sqrt((wc/wlagh*wc/wlagh+1)/(wc/wlagl*wc/wlagl+1));
//   double glead=sqrt((wc/wleadl*wc/wleadl+1)/(wc/wleadh*wc/wleadh+1));
//   LTISys lead(sleadb,1,sleada,1,1/glead);
//   LTISys lag(slagb,1,slaga,1,1/glag);
//   leadlag=lead*lag;
//   }
//   //resonance compensation: supress leakage of high frequency noise
//   LTISys reso(1);
//   if(fpara[mid][0]>0&&fpara[mid][2]>0)
//   {
//   double w1=fpara[mid][0]*2*M_PI;
//   double zeta1=fpara[mid][1];
//   double w2=fpara[mid][2]*2*M_PI;
//   double zeta2=fpara[mid][3];
//   double sa1[]={1,2*zeta1/w1,1/w1/w1};
//   double sb1[]={1,2*zeta2/w2,1/w2/w2};   
//   reso=LTISys(sb1,2,sa1,2);
//   }
//   LTISys overall(reso*leadlag);  
//   
//   LTISys *tmp=filter[mid];  
//   filter[mid]=new LTISys(overall.tustin(ts));  
//   if(tmp->order_b>0)
//     filter[mid]->initialize(tmp->x[0]);
//   delete tmp;
//   
//   //delay compensation for input.
//   LTISys delay(1);
//   if(fpara[mid][6]>0)
//   {
//   double td=fpara[mid][6]*1e-3;
//   double sdelayb[]={1,td};
//   double sdelaya[]={1,td/10};
//   delay=LTISys(sdelayb,1,sdelaya,1);
//   }
//   tmp=flt_delay[mid];
//   flt_delay[mid]=new LTISys(delay.tustin(ts));
//   if(tmp->order_b>0)
//     flt_delay[mid]->initialize(tmp->x[0]);
// //  cout<<*flt_delay;
//   delete tmp;
//   
// }

void TrajectoryController::set_position_ref()
{
  for (int i=0;i<4;i++)
  {
    mzero[i]=m[i];
    qzero[i]=q[i];

  }
}
void TrajectoryController::set_gc()
{
  for (int i=0;i<4;i++)
  {
    tzero[i]=mt[i];
  }
}
void TrajectoryController::clear_ref()
{
  for (int i=0;i<4;i++)
  {
    mzero[i]=0;
    qzero[i]=0;
    tzero[i]=0;
  }
}

//calculate the frames given the joint values.
void TrajectoryController::updateKinematics()
{
  frm_cur[0]=frm_home[0];
  frm0[0]=frm_home[0];
  for (int i=1;i<=4;i++)
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
  for (int i=3;i>=0;i--)
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
    qv[i]=0;
    dqv[i]=0;
    dq[i]=0;
    qt[i]=0;
    qd[i]=q[i];
    qvd[i]=qv[i];

    m[i]=rob->q[i];
    mv[i]=0;
    md[i]=m[i];
    mvd[i]=mv[i];
    dm[i]=0;
    dmv[i]=0;
    mt[i]=0;
  }
  for (int i=0;i<4;i++)
    gc[i]=Vector(constants::gravc[i*2],constants::gravc[i*2+1],0);

  state=SERVO;
  //ctr_mode=GRAVITY;
  ctr_mode=GC_M2JT;
  //ctr_mode=PDWITHGC;

}







// void TrajectoryController::vstep(int joint, double step)
// {
//   vector<double> des(4,0);
//   for (int i=0;i<4;i++)
//     des[i]=qd[i];
//   des[joint]+=step;
//   cur_act=new MoveAbsAction(des,this);
//   cur_act->initialize();
//
// }

void TrajectoryController::qstep(int joint, double step)
{

//   for (int i=0;i<4;i++)
//     qd[i]=q[i];
  qd[joint]+=step;

}
 
void TrajectoryController::mstep(int joint, double step)
{

//   for (int i=0;i<4;i++)
//     md[i]=m[i];
  md[joint]+=step;

}


void TrajectoryController::update()
{
  rob->update();
  for (int i=0;i<4;i++)
  {
    m[i]=rob->q[i];
    mv[i]=rob->v[i];
  }
  clock_gettime(CLOCK_REALTIME, &ts);
  //save the last joint position
//   for (int i=0;i<4;i++)
//     ql[i]=q[i];
  rob->motor2JointPosition(rob->q,q);
  rob->motor2JointVelocity(rob->v,qv);

  if (do_qvstep)
      do_qVStep();
    else if (do_mvstep)
      do_mVStep();
    else if (do_qsin)
      do_qSin();
//     else if (do_msin)
//       do_mSin();
    else if (do_chirp)
      //do_motorSine();
      md_Km_flt();
      //do_ChirpSine();
      //do_stretch();
      
    if (ctr_mode==QD_KQ_M2JT)
      qd_Kq_m2jT();
    else if (ctr_mode==QD_KQ_GC_M2JT)
      qd_Kq_gc_m2jT();
    else if (ctr_mode==QD_J2M_KM)
      qd_j2m_Km();
    else if (ctr_mode==GC_M2JT)
      gc_m2jT();
    else if (ctr_mode==MD_KM)
      md_VarKm();
    else if (ctr_mode==NOCONTROL)
    {
      for (int i=0;i<4;i++)
        rob->torque[i]=tzero[i];
    }
    else if (ctr_mode==SYSID)
    {
      for (int i=0;i<4;i++)
        rob->torque[i]=mt[i];
    }
    
    logging();
  
}

void TrajectoryController::do_stretch()
{
  for (int i=0;i<4;i++)
    mt[i]=tzero[i];

  if (mtvstep<mvtime[jid])
  {
    mt[jid]-=mtvstep*0.15/5000-0.02;
    mtvstep++;
    if (mtvstep%100==0)
      cout<<mt[jid]<<endl;
  }
}



void TrajectoryController::do_ChirpSine()
{


  double t=double(mtvstep)/double(constants::cycle_hz);
  double freq=chirp_step;
  for (int i=0;i<4;i++)
  {
    mt[i]=tzero[i];
    md[i]=mzero[i];
  }
  mt[jid]+=mvstep[jid]*sin(2*M_PI*freq*t);
  mtvstep++;
//     if(mtvstep%1000==0)
//       cout<<freq<<"Hz"<<endl;


}


// void TrajectoryController::md_Km_flt()
// {
//   double t=double(mtvstep)/double(constants::cycle_hz);
//   double freq=chirp_step;
//   for (int i=0;i<4;i++)
//   {
//     mt[i]=tzero[i];
//     md[i]=mzero[i];
//     dmv[i]=-mv[i];
//   }
// 
//   md[jid]+=mvstep[jid]*sin(2*M_PI*freq*t);
//   
//   for(int i=0;i<4;i++)
//   {
//     int jid=i;
//   if(mtvstep==0)
//     flt_delay[jid]->initialize(m[jid]);
//   if(close_loop)
//   {
//     qd[jid]=flt_delay[jid]->filter(m[jid]);
//     //double tmp=dm[i];
//     dm[jid]=md[jid]-qd[jid];
//     //dmv[i]=0-mv[i];
//     dq[jid]=filter[jid]->filter(dm[jid]);
//     
//     double dis=fabs(dq[jid]);
//     double f0=mvtime[jid]*1e-3;
//     double d0=f0/Kmp[jid];
//     double d1=300*d0;
//     double f1=2*f0;
//     double c=3.0;
//     if(dis<d0)
//       mt[jid]+=Kmp[jid]*dq[jid];
//     else if(dis<d1&&dq[jid]>0)
//       mt[jid]+=f0+(f1-f0)*(exp(-c*d0)-exp(-c*dis))/(exp(-c*d0)-exp(-c*d1));
//     else if(dis<d1&&dq[jid]<0)
//       mt[jid]-=f0+(f1-f0)*(exp(-c*d0)-exp(-c*dis))/(exp(-c*d0)-exp(-c*d1));
//     else if(dq[jid]>0)
//       mt[jid]+=f1;
//     else
//       mt[jid]-=f1;
//     
//     //cout<<m[jid]<<','<<qd[jid]<<','<<dm[jid]<<','<<dq[jid]<<','<<mt[jid]<<endl;
//   }
//   else
//   {
//     
//     dm[jid]=md[jid]-mzero[jid];  
//     dq[jid]=filter[jid]->filter(dm[jid]);
//     qd[jid]=flt_delay[jid]->filter(m[jid]);
//     //dq[jid]=dm[jid];
//     mt[jid]+=Kmp[jid]*dq[jid];
//   }
//   }
//   //mt[jid]+=mvstep[jid]*sin(2*M_PI*freq*t);
//   mtvstep++;
// }

void TrajectoryController::md_Km_flt()
{
  double t=double(mtvstep)/double(constants::cycle_hz);
  double freq=chirp_step;
  for (int i=0;i<4;i++)
  {
    mt[i]=tzero[i];
    md[i]=mzero[i];
    dmv[i]=-mv[i];
  }

  md[jid]+=mvstep[jid]*sin(2*M_PI*freq*t);
  
  for(int i=0;i<4;i++)
  {
    int jid=i;
  if(mtvstep==0)
    flt_delay[jid]->initialize(m[jid]);
  if(close_loop)
  {
    qd[jid]=flt_delay[jid]->filter(m[jid]);
    //double tmp=dm[i];
    dm[jid]=md[jid]-qd[jid];
    //dmv[i]=0-mv[i];
    dq[jid]=filter[jid]->filter(dm[jid]);
    
    double r=Kmp[jid]*dq[jid]/(mvtime[jid]*1e-3);
    if (fabs(r)<=1)
      mt[jid]+=Kmp[jid]*dq[jid];
    else if(fabs(r)<=8)
    {
      double mt1=Kmp[jid]*dq[jid];
      double mt2=Kmp[jid]/8*dq[jid]+Kmv[jid]*dmv[jid];
      double ratio= 1/(1+10*exp(fabs(r)-1)*exp(fabs(r)-1));
      mt[jid]+=mt1*ratio+mt2*(1-ratio);
    }
    else if (r>8)
    {
      double mt1=Kmp[jid]/8*dq[jid]+Kmv[jid]*dmv[jid];
      double mt2=max_torque+Kmv[jid]/(1+log(fabs(r/8)))*dmv[jid];
      double ratio= 1/(1+10*exp(fabs(r/8)-1)*exp(fabs(r/8)-1));
      mt[jid]+=mt1*ratio+mt2*(1-ratio);
      //mt[jid]=max_torque+Kmv[jid]/(1+log(fabs(r)))*dmv[jid];
    }
    else
    {
      double mt1=Kmp[jid]/8*dq[jid]+Kmv[jid]*dmv[jid];
      double mt2=-max_torque+Kmv[jid]/(1+log(fabs(r/8)))*dmv[jid];
      double ratio= 1/(1+10*exp(fabs(r/8)-1)*exp(fabs(r/8)-1));
      mt[jid]+=mt1*ratio+mt2*(1-ratio);
      //mt[jid]=-max_torque+Kmv[jid]/(1+log(fabs(r)))*dmv[jid];
    }
    //mt[jid]+=Kmp[jid]*dq[jid];
    //cout<<m[jid]<<','<<qd[jid]<<','<<dm[jid]<<','<<dq[jid]<<','<<mt[jid]<<endl;
  }
  else
  {
    
    dm[jid]=md[jid]-mzero[jid];  
    dq[jid]=filter[jid]->filter(dm[jid]);
    qd[jid]=flt_delay[jid]->filter(m[jid]);
    //dq[jid]=dm[jid];
    mt[jid]+=Kmp[jid]*dq[jid];
  }
  }
  //mt[jid]+=mvstep[jid]*sin(2*M_PI*freq*t);
  mtvstep++;
}


void TrajectoryController::do_motorSine()
{
  double t=double(mtvstep)/double(constants::cycle_hz);
  double freq=chirp_step;
  for (int i=0;i<4;i++)
  {
    mt[i]=tzero[i];
    md[i]=mzero[i];
    dmv[i]=-mv[i];
  }

  md[jid]+=mvstep[jid]*sin(2*M_PI*freq*t);
  

  if(mtvstep==0)
    flt_delay[jid]->initialize(m[jid]);
  if(close_loop)
  {
    qd[jid]=flt_delay[jid]->filter(m[jid]);
    //double tmp=dm[i];
    dm[jid]=md[jid]-qd[jid];
    //dmv[i]=0-mv[i];
    dq[jid]=filter[jid]->filter(dm[jid]);
    
    double r=Kmp[jid]*dq[jid]/max_torque;
    if (fabs(r)<=1)
      mt[jid]+=Kmp[jid]*dq[jid];
    else if (r>1)
      //mt[jid]=max_torque+Kmv[jid]/(1+log(fabs(r)))*dmv[jid];
      mt[jid]+=Kmp[jid]*dq[jid]/fabs(r);
    else
      mt[jid]+=Kmp[jid]*dq[jid]/fabs(r);
      //mt[jid]=-max_torque+Kmv[jid]/(1+log(fabs(r)))*dmv[jid];
    
    //mt[jid]+=Kmp[jid]*dq[jid];
    //cout<<m[jid]<<','<<qd[jid]<<','<<dm[jid]<<','<<dq[jid]<<','<<mt[jid]<<endl;
  }
  else
  {
    
    dm[jid]=md[jid]-mzero[jid];  
    dq[jid]=filter[jid]->filter(dm[jid]);
    qd[jid]=flt_delay[jid]->filter(m[jid]);
    //dq[jid]=dm[jid];
    mt[jid]+=Kmp[jid]*dq[jid];
  }
  
  //mt[jid]+=mvstep[jid]*sin(2*M_PI*freq*t);
  mtvstep++;
}


void TrajectoryController::do_qSin()
{

  if (qtvstep==2*qvtime[jid])
    qtvstep=0;
  double tmp=qzero[jid]+qvstep[jid]*sin(double(qtvstep)/double(qvtime[jid])*M_PI);
  qvd[jid]=tmp-qd[jid];
  qd[jid]=tmp;
  qtvstep++;

}

void TrajectoryController::do_Chirp()
{

//   updateKinematics();
//   Vector g(0,0,-1);
//   //double tq[4];
//   for (int i=0;i<4;i++)
//   {
//     Vector Zi=frm0[i+1].M.UnitZ();
//     qt[i]=0;
//     for (int j=i;j<4;j++)
//     {
//       Rotation R0j=frm0[j+1].M.Inverse();
//       Vector tmp=R0j*(g*Zi);
//       qt[i]-=dot(gc[j],tmp);
//     }
//   }
//
//   for (int i=0;i<4;i++)
//   {
//     mt[i]=0;
//     for (int j=0;j<4;j++)
//     {
//       mt[i]+=rob->m2j[4*j+i]*qt[j];
//
//     }
//
//   }
//   if (mtvstep==0)
//     for (int j=0;j<4;j++)
//       for (int i=0;i<filter_order;i++)
//       {
//         tx[j][i]=tzero[j];
//         ty[j][i]=tzero[j];
//       }
  double t=double(mtvstep)/double(constants::cycle_hz);
  double freq=exp(chirp_step*t*5);
  for (int i=0;i<4;i++)
    mt[i]=tzero[i];
  if (mtvstep<mvtime[jid])
  {
    mt[jid]+=mvstep[jid]*sin(2*M_PI*freq*t);
    mtvstep++;
    if (mtvstep%1000==0)
      cout<<freq<<"Hz"<<endl;
  }
//   for(int i=0;i<4;i++)
//     qt[i]=mt[i];
//   double tmp[4];
//   for(int i=2;i<=2;i++)
//   {
//     tmp[i]=b[0]*mt[i];
//     for(int j=0;j<filter_order;j++)
//       tmp[i]+=b[j+1]*tx[i][j];
//     for(int j=0;j<filter_order;j++)
//       tmp[i]-=a[j+1]*ty[i][j];
//     for(int j=filter_order-1;j>0;j--)
//       tx[i][j]=tx[i][j-1];
//     tx[i][0]=mt[i];
//     for(int j=filter_order-1;j>0;j--)
//       ty[i][j]=ty[i][j-1];
//     ty[i][0]=tmp[i];
//     //q[i]=tmp[i];
//     mt[i]=ty[i][0];
//   }

//   if(mtvstep>mvtime[jid])
//     do_chirp=false;
}





void TrajectoryController::do_qVStep()
{
  if (qtvstep==2*qvtime[jid])
    qtvstep=0;
  if (qtvstep<qvtime[jid]/2)
  {
    qd[jid]+=qvstep[jid]*2/qvtime[jid];
    qvd[jid]=qvstep[jid]*2/qvtime[jid];
  }
  else if (qtvstep<qvtime[jid]/2+qvtime[jid])
  {
    qd[jid]-=qvstep[jid]*2/qvtime[jid];
    qvd[jid]=-qvstep[jid]*2/qvtime[jid];
  }
  else
  {
    qd[jid]+=qvstep[jid]*2/qvtime[jid];
    qvd[jid]=qvstep[jid]*2/qvtime[jid];
  }
  qtvstep++;
}

void TrajectoryController::do_mVStep()
{
  if (mtvstep==2*mvtime[jid])
    mtvstep=0;
  if (mtvstep<mvtime[jid]/2)
  {
    md[jid]+=mvstep[jid]*2/mvtime[jid];
    mvd[jid]=mvstep[jid]*2/mvtime[jid];
  }
  else if (mtvstep<mvtime[jid]/2+mvtime[jid])
  {
    md[jid]-=mvstep[jid]*2/mvtime[jid];
    mvd[jid]=-mvstep[jid]*2/mvtime[jid];
  }
  else
  {
    md[jid]+=mvstep[jid]*2/mvtime[jid];
    mvd[jid]=mvstep[jid]*2/mvtime[jid];
  }
  mtvstep++;
}

void TrajectoryController::gc_m2jT()
{

  updateKinematics();
  //T=sum(gcj) dot (g*Zi
  Vector g(0,0,-1);
  //double tq[4];
  for (int i=0;i<4;i++)
  {
    Vector Zi=frm0[i+1].M.UnitZ();
    qt[i]=0;
    for (int j=i;j<4;j++)
    {
      Rotation R0j=frm0[j+1].M.Inverse();
      Vector tmp=R0j*(g*Zi);
      qt[i]-=dot(gc[j],tmp);
    }
  }

  for (int i=0;i<4;i++)
  {
    mt[i]=0;
    for (int j=0;j<4;j++)
    {
      mt[i]+=rob->m2j[4*j+i]*qt[j];

    }

  }
  for (int i=0;i<4;i++)
    rob->torque[i]=mt[i];
  //logging();

}

void TrajectoryController::qd_Kq_gc_m2jT()
{
  updateKinematics();
  for (int i=0;i<4;i++)
  {

    dq[i]=qd[i]-q[i];
    dqv[i]=qvd[i]-qv[i];
    qt[i]=Kqp[i]*dq[i]+Kqv[i]*dqv[i];
  }

  //T=sum(gcj) dot (g*Zi
  Vector g(0,0,-1);
  //double tq[4];
  for (int i=0;i<4;i++)
  {
    Vector Zi=frm0[i+1].M.UnitZ();
    for (int j=i;j<4;j++)
    {
      Rotation R0j=frm0[j+1].M.Inverse();
      Vector tmp=R0j*(g*Zi);
      qt[i]-=dot(gc[j],tmp);
    }
  }
  for (int i=0;i<4;i++)
  {
    mt[i]=0;
    for (int j=0;j<4;j++)
    {
      mt[i]+=rob->m2j[4*j+i]*qt[j];

    }

  }
  for (int i=0;i<4;i++)
    rob->torque[i]=mt[i];
  //logging();
}

void TrajectoryController::md_VarKm()
{
  for (int i=0;i<4;i++)
  {
    dm[i]=md[i]-m[i];
    dmv[i]=mvd[i]-mv[i];
    //dv[i]=-v[i];
  }


  for (int i=0;i<4;i++)
  {
    double r=Kmp[i]*dm[i]/max_torque;
    if (fabs(r)<=1)
      mt[i]=Kmp[i]*dm[i]+Kmv[i]*dmv[i];
    else if (r>1)
      mt[i]=max_torque+Kmv[i]/(1+log(fabs(r)))*dmv[i];
    else
      mt[i]=-max_torque+Kmv[i]/(1+log(fabs(r)))*dmv[i];
  }
  for (int i=0;i<4;i++)
    rob->torque[i]=mt[i];
}

void TrajectoryController::md_Km_VarKm_Comp()
{

  for (int i=0;i<4;i++)
  {
    dm[i]=md[i]-m[i];
    dmv[i]=mvd[i]-mv[i];
    //dv[i]=-v[i];
  }

  for (int i=0;i<4;i++)
  {
    if (fabs(Kmp[i]*dm[i])<=max_torque)
      mt[i]=Kmp[i]*dm[i]+Kmv[i]*dmv[i];
    else if (Kmp[i]*dm[i]>max_torque)
      mt[i]=max_torque+Kmv[i]*dmv[i];
    else
      mt[i]=-max_torque+Kmv[i]*dmv[i];
  }

  for (int i=2;i<=2;i++)
  {
    if (fabs(Kmp[i]*dm[i])<=max_torque)
      mt[i]=Kmp[i]*dm[i];
    else if (Kmp[i]*dm[i]>max_torque)
      mt[i]=max_torque;
    else
      mt[i]=-max_torque;
  }

//   double tmp[4];
//   for (int i=2;i<=2;i++)
//   {
//     tmp[i]=b[0]*mt[i];
//     for (int j=0;j<filter_order;j++)
//       tmp[i]+=b[j+1]*tx[i][j];
//     for (int j=0;j<filter_order;j++)
//       tmp[i]-=a[j+1]*ty[i][j];
//     for (int j=filter_order-1;j>0;j--)
//       tx[i][j]=tx[i][j-1];
//     tx[i][0]=mt[i];
//     for (int j=filter_order-1;j>0;j--)
//       ty[i][j]=ty[i][j-1];
//     ty[i][0]=tmp[i];
//     //q[i]=tmp[i];
//     mt[i]=ty[i][0];
//   }

  for (int i=0;i<4;i++)
    rob->torque[i]=mt[i];

}

void TrajectoryController::md_Km()
{
  for (int i=0;i<4;i++)
  {
    dm[i]=md[i]-m[i];
    dmv[i]=mvd[i]-mv[i];
    //dv[i]=-v[i];
  }


  for (int i=0;i<4;i++)
  {
    mt[i]=Kmp[i]*dm[i]+Kmv[i]*dmv[i];
  }
  for (int i=0;i<4;i++)
    rob->torque[i]=mt[i];
  //logging();
}

void TrajectoryController::qd_j2m_Km()
{
  for (int i=0;i<4;i++)
  {
    dq[i]=qd[i]-q[i];
    dqv[i]=qvd[i]-qv[i];
    //dv[i]=-v[i];
  }

  //transform errors into motor space

  for (int i=0;i<4;i++)
  {
    dm[i]=0;
    dmv[i]=0;
    for (int j=0;j<4;j++)
    {
      dm[i]+=dq[j]*rob->j2m[4*i+j];
      dmv[i]+=dqv[j]*rob->j2m[4*i+j];
    }
  }

  for (int i=0;i<4;i++)
  {
    mt[i]=Kmp[i]*dm[i]+Kmv[i]*dmv[i];
  }
  for (int i=0;i<4;i++)
    rob->torque[i]=mt[i];
  //logging();
}

void TrajectoryController::qd_Kq_m2jT()
{
  for (int i=0;i<4;i++)
  {

    dq[i]=qd[i]-q[i];
    qt[i]=Kqp[i]*dq[i]-Kqv[i]*(qv[i]-qvd[i]);
  }
  for (int i=0;i<4;i++)
  {
    mt[i]=0;
    for (int j=0;j<4;j++)
    {
      mt[i]+=rob->m2j[4*j+i]*qt[j];

    }

  }
  for (int i=0;i<4;i++)
    rob->torque[i]=mt[i];
  //logging();
}

void TrajectoryController::logging()
{
  std::string log;
  std::stringstream out;
  for (int i=0;i<4;i++)
    out<<m[i]-mzero[i]<<",";
  for (int i=0;i<4;i++)
    out<<md[i]-mzero[i]<<",";
  for (int i=0;i<4;i++)
    out<<mv[i]<<",";
  for (int i=0;i<4;i++)
    out<<mt[i]<<",";
  for (int i=0;i<4;i++)
    out<<dm[i]<<",";
  for (int i=0;i<4;i++)
    out<<q[i]-qzero[i]<<",";
  for (int i=0;i<4;i++)
    out<<qd[i]-mzero[i]<<",";
  for (int i=0;i<3;i++)
    out<<qv[i]<<",";
  out<<rob->dt<<',';
  for (int i=0;i<4;i++)
    out<<qt[i]<<",";
  for (int i=0;i<4;i++)
    out<<dq[i]<<",";
  
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
    js.velocity[i]=qv[i];
    js.effort[i]=qt[i];
  }
  joint_state_pub.publish(js);
}

// void TrajectoryController::logging()
// {
//   std::string log;
//   std::stringstream out;
//   for (int i=0;i<4;i++)
//     out<<q[i]<<",";
//   for (int i=0;i<4;i++)
//     out<<qd[i]<<",";
//   for (int i=0;i<4;i++)
//     out<<v[i]<<",";
//   for (int i=0;i<4;i++)
//     out<<f[i]<<",";
//   for (int i=0;i<4;i++)
//     out<<dq[i]<<",";
//   for (int i=0;i<4;i++)
//     out<<rob->q[i]<<',';
//   for (int i=0;i<4;i++)
//     out<<rob->v[i]<<',';
//   for (int i=0;i<4;i++)
//     out<<rob->torque[i]<<',';
//   out << ts.tv_sec << ','<< ts.tv_nsec;
//   log = out.str();
//   diag.data=log;
//   diag.header.stamp.sec=ts.tv_sec;
//   diag.header.stamp.nsec=ts.tv_nsec;
//   diagnostic_pub.publish(diag);
//   //vector<double> cur_pos_act(q,q+4);
//   js.header.stamp.sec=ts.tv_sec;
//   js.header.stamp.nsec=ts.tv_nsec;
//   //js.position=cur_pos_act;
//
//   for (int i=0;i<4;i++)
//   {
//     js.position[i]=q[i];
//     js.velocity[i]=v[i];
//     js.effort[i]=f[i];
//   }
//   joint_state_pub.publish(js);
// }
void TrajectoryController::stop()
{
  rob->close();
}


