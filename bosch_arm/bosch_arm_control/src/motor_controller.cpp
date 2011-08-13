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
#include "motor_controller.h"
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
  ///resonance compensation y1=1/4,f1=8,y2=1/8,f2=40
//   filter_order=2;
//   double bb[]={4.134265039735645e-02,    -7.771105812573594e-02,     3.886211445065686e-02};
//   double aa[]={1.000000000000000e+00,    -1.972700933810727e+00,     9.751946405330041e-01};
///resonance compensation y1=1/2,f1=8,y2=1/8,f2=40, lead compensation f3=1, f4=10
//   filter_order=3;
//   double bb[]={3.971672532347112e-01,    -1.141227944523378e+00,     1.115209779749842e+00,-3.709990377268638e-01};
//   double aa[]={1.000000000000000e+00,    -2.887615836095562e+00,     2.780830044410648e+00,-8.930641575807744e-01};
//   a=new double[filter_order+1];
//   b=new double[filter_order+1];
//   for (int i=0;i<filter_order+1;i++)
//   {
//     a[i]=aa[i];
//     b[i]=bb[i];
//   }
//   for (int i=0;i<4;i++)
//   {
//     tx[i]=new double[filter_order];
//     ty[i]=new double[filter_order];
//   }
//   for (int j=0;j<4;j++)
//     for (int i=0;i<filter_order;i++)
//     {
//       tx[j][i]=tzero[j];
//       ty[j][i]=tzero[j];
//     }
  qtvstep=0;
  mtvstep=0;
  max_torque=0.05;
  clear_ref();

  for(int i=0;i<4;i++)
    filter[i]=new LTISys(1);
  
  flt_delay=new LTISys(1);
  
  fpara[0]=25;
  fpara[1]=0.2;
  fpara[2]=55;
  fpara[3]=0.2;
  fpara[4]=50;
  fpara[5]=0.3;
  Kmp[3]=6;
  
  
  updateFilter();
//   double w1=25*2*M_PI;
//   double w2=55*2*M_PI;
//   double zeta1=1.0/4;
//   double zeta2=1.0/8;
//   double rescpa[]={1,2*zeta1/w1,1/w1/w1};
//   double rescpb[]={1,2*zeta2/w2,1/w2/w2};
//   LTISys rescp(rescpb,2,rescpa,2);
//   LTISys rescpd=rescp.tustin(0.001);

//   double w1=4*2*M_PI;
//   double w2=50*2*M_PI;
//   double sb[]={1,1/w1};
//   double sa[]={1,1/w2};
//   LTISys sys(sb,1,sa,1);
//   sys=sys.tustin(0.001);
//   for(int i=0;i<4;i++)
//     filter[i]=new LTISys(sys);
//   cout<<sys.b[1]<<','<<sys.b[0]<<endl;
//   cout<<sys.a[1]<<','<<sys.a[0]<<endl;
  //cout<<rescpd<<endl;
//  LTISys *ptrpp=&rescpd;


}

// void TrajectoryController::updateFilter()
// {
//   double w1=fpara[0]*2*M_PI;
//   double zeta1=fpara[1];
//   double w2=fpara[2]*2*M_PI;
//   double zeta2=fpara[3];
//   double w3=fpara[4]*fpara[5]*2*M_PI;
//   double w4=fpara[4]/fpara[5]*2*M_PI;
//   double sa1[]={1,2*zeta1/w1,1/w1/w1};
//   double sb1[]={1,2*zeta2/w2,1/w2/w2};  
//   double wcgain=sqrt((1+1/fpara[5]/fpara[5])/(1+fpara[5]*fpara[5]));
//   //cout<<' '<<wcgain<<' '<<endl;
//   double sa2[]={1,1/w4};
//   double sb2[]={1/wcgain,1/w3/wcgain};
//   double td=2.3e-3;
//   double ts=1e-3;
//   double sb3[]={1,td/2};
//   double sa3[]={1,-td/2};
//   LTISys sys3(sb3,1,sa3,1);
//   LTISys sys_delay=sys3.tustin(ts);
//   flt_delay=new LTISys(sys_delay);
//   flt_delay->initialize(m[3]);
//   
//   LTISys sys1(sb1,2,sa1,2);
//   LTISys sys2(sb2,1,sa2,1);
// //   double t1[]={1};
// //   LTISys sys3(t1,0,sa2,1);
//   LTISys sys=sys1*sys2;
//   sys=sys.tustin(ts);
//   if(filter[3]->order_b>0)
//     sys.initialize(filter[3]->x[0]);
//   LTISys *tmp=filter[3];
//   
//   filter[3]=new LTISys(sys);
//   
//   delete tmp;
//   //cout<<sys<<endl;
// }


void TrajectoryController::updateFilter()
{
  double wc=fpara[4]*2*M_PI;
  double ts=1.0/constants::cycle_hz;
  double alpha=1.0/fpara[5];
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
  LTISys lead(sleadb,1,sleada,1,1/glead);
  LTISys lag(slagb,1,slaga,1,1/glag);
  LTISys leadlag(lead*lag);
  
  //resonance compensation 
  double w1=fpara[0]*2*M_PI;
  double zeta1=fpara[1];
  double w2=fpara[2]*2*M_PI;
  double zeta2=fpara[3];
  double sa1[]={1,2*zeta1/w1,1/w1/w1};
  double sb1[]={1,2*zeta2/w2,1/w2/w2};   
  LTISys reso(sb1,2,sa1,2);
  LTISys overall(reso*leadlag);  
  
  LTISys *tmp=filter[3];  
  filter[3]=new LTISys(overall.tustin(ts));  
  if(tmp->order_b>0)
    filter[3]->initialize(tmp->x[0]);
  delete tmp;
  
  //delay compensation for input.
  double td=2.3e-3;
  double sdelayb[]={1,td};
  double sdelaya[]={1,td/10};
  LTISys delay(sdelayb,1,sdelaya,1);
  tmp=flt_delay;
  flt_delay=new LTISys(delay.tustin(ts));
  if(tmp->order_b>0)
    flt_delay->initialize(tmp->x[0]);
  cout<<*flt_delay;
  delete tmp;
  
}

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
  //state=CALIBRATION;
  //state=LOADTRACE;
  state=SERVO;
  //ctr_mode=GRAVITY;
  ctr_mode=NOCONTROL;
  //ctr_mode=PDWITHGC;
  if (state==CALIBRATION)
  {
    genCalibTraj2();
    cur_act=act_que.front();
    cur_act->initialize();
    act_que.pop_front();
  }
  else if (state==LOADTRACE)
  {

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
  for (int j=0;j<calib1->len_b;j++)
    if (calib1->count[j]>0&&calib2->count[j]>0)
    {
      calib1->b[j]-=calib2->b[j];
      calib1->count[j]=1;
    }

  for (int i=0;i<8;i++)
  {
    for (int j=0;j<calib1->len_b;j++)
    {
      if (calib1->count[j]>0)
      {
        calib1->A[i][j]-=calib2->A[i][j];
      }
    }
  }

  Eigen::MatrixXf AA(8,8);
  for (int i=0;i<8;i++)
  {
    for (int j=0;j<8;j++)
    {
      double s=0;
      for (int k=0;k<calib1->len_b;k++)
        if (calib1->count[k]>0)
          s+=calib1->A[i][k]*calib1->A[j][k];
      AA(i,j)=s/calib1->len_b;
    }
  }
  //cout<<AA;
  Eigen::VectorXf Ab(8);
  for (int i=0;i<8;i++)
  {
    double s=0;
    for (int j=0;j<calib1->len_b;j++)
      if (calib1->count[j]>0)
        s+=calib1->A[i][j]*calib1->b[j];
    Ab(i)=-s/calib1->len_b;
  }
  Eigen::MatrixXf M=AA.inverse()*Ab;
  //cout<<M<<endl;
  for (int i=0;i<4;i++)
  {
    gc[i]=Vector(M(i*2,0),M(i*2+1,0),0);
  }
  for (int i=0;i<4;i++)
  {
    for (int j=0;j<3;j++)
      cout<<gc[i][j]<<',';
    cout<<endl;
  }
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

  if (state==CALIBRATION)
  {
    //update desired position
    if (cur_act->finished)
    {
      if (cur_act->calib==NULL)
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
      qvd[i]=cur_act->cmd.velocity[i];

    }
    //update torque command
    qd_Kq_gc_m2jT();

    logging();
  }
  else if (state==LOADTRACE)
  {
    if (cur_act->finished)
    {
      if (cur_act->calib==NULL)
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
      qvd[i]=cur_act->cmd.velocity[i];

    }
    //update torque command
    qd_Kq_gc_m2jT();
    //PDControl();
    logging();
  }
  else if (state==SERVO)
  {
    if (cur_act!=NULL)
    {
      if (cur_act->finished)
      {
        if (cur_act->calib==NULL)
        {
          delete cur_act;
          cur_act=NULL;
        }

        if (act_que.size()!=0)
        {
          cur_act=act_que.front();
          cur_act->initialize();
          act_que.pop_front();
        }
        else
          cur_act=NULL;

      }
      else
      {
        cur_act->update();
        for (int i=0;i<4;i++)
        {
          qd[i]=cur_act->cmd.coordinates[i];
          qvd[i]=cur_act->cmd.velocity[i];

        }
      }
    }
    else if (act_que.size()!=0)
    {
      cur_act=act_que.front();
      cur_act->initialize();
      act_que.pop_front();
    }
    else if (do_qvstep)
      do_qVStep();
    else if (do_mvstep)
      do_mVStep();
    else if (do_qsin)
      do_qSin();
//     else if (do_msin)
//       do_mSin();
    else if (do_chirp)
      do_motorSine();
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
  
    
//   if(mtvstep%500<250)
//     md[jid]+=mvstep[jid];
//   else
//     md[jid]-=mvstep[jid];
//   if(close_loop)
//   {
//     dm[jid]=md[jid]-m[jid];
//     dq[jid]=filter[jid]->filter(dm[jid]);
//     double dmt=Kmp[jid]*dq[jid];
//     if(fabs(dmt)<0.06)
//       mt[jid]+=Kmp[jid]*dq[jid];
//     else if(dmt>=0.06)
//       mt[jid]+=0.06;
//     else
//       mt[jid]-=0.06;
//   }
//   if(close_loop)
//   {
//     dm[jid]=md[jid]-m[jid];
//     double dmt=Kmp[jid]*dm[jid];
//     if(dmt>0.06)
//       dm[jid]=0.06/Kmp[jid];
//     else if(dmt<-0.06)
//       dm[jid]=-0.06/Kmp[jid];
// 
//     dq[jid]=filter[jid]->filter(dm[jid]);
//     mt[jid]+=Kmp[jid]*dq[jid];
//   }
  //Kmp[3]=2;
  if(mtvstep==0)
    flt_delay->initialize(m[jid]);
  if(close_loop)
  {
    qd[jid]=flt_delay->filter(m[jid]);
    //double tmp=dm[i];
    dm[jid]=md[jid]-qd[jid];
    //dmv[i]=0-mv[i];
    dq[jid]=filter[jid]->filter(dm[jid]);
    
    double r=Kmp[jid]*dq[jid]/max_torque;
    if (fabs(r)<=1)
      mt[jid]+=Kmp[jid]*dq[jid];
//     else if(fabs(r)<=20)
//       mt[jid]+=Kmp[jid]*dm[jid]/20+Kmv[jid]*dmv[jid];
    else if (r>1)
      mt[jid]=max_torque+Kmv[jid]/(1+log(fabs(r)))*dmv[jid];
    else
      mt[jid]=-max_torque+Kmv[jid]/(1+log(fabs(r)))*dmv[jid];
    
    //mt[jid]+=Kmp[jid]*dq[jid];
    //cout<<m[jid]<<','<<qd[jid]<<','<<dm[jid]<<','<<dq[jid]<<','<<mt[jid]<<endl;
  }
  else
  {
    
    dm[jid]=md[jid]-mzero[jid];  
    //dq[jid]=filter[jid]->filter(dm[jid]);
    //qd[jid]=flt_delay->filter(m[jid]);
    dq[jid]=dm[jid];
    mt[jid]+=Kmp[jid]*dq[jid];
  }
  
  //mt[jid]+=mvstep[jid]*sin(2*M_PI*freq*t);
  mtvstep++;
}


// void TrajectoryController::do_mSin()
// {
//   double t=double(mtvstep)/double(constants::cycle_hz);
//   double freq=chirp_step;
//   for (int i=0;i<4;i++)
//   {
//    md[i]=mzero[i];
//    mt[i]=tzero[i];
//  }
//   md[jid]+=mvstep[jid]*sin(2*M_PI*freq*t);
   
  //double tmp=dm[jid];

  //dmv[jid]=(dm[jid]-tmp)/rob->dt;
  //dq[jid]=filter[jid]->filter(dm[jid]);
  //dq[jid]=dm[jid];
  //mt[jid]+=Kmp[jid]*dm[jid];
  //mt[jid]+=dm[jid];
  
//   mt[jid]+=mvstep[jid]*sin(2*M_PI*freq*t);
//   mtvstep++;
//   if (mtvstep==2*mvtime[jid])
//         mtvstep=0;
//   double tmp=mzero[jid]+mvstep[jid]*sin(double(mtvstep)/(double)mvtime[jid]*M_PI);
//   mvd[jid]=tmp-md[jid];
//   md[jid]=tmp;
//   mtvstep++;
//}

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
    out<<mt[i]-tzero[i]<<",";
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
    b[bin*4+i]+=ctr->qt[i];
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


LTISys::LTISys(double c)
{
  a=new double[1];
  b=new double[1];
  x=new double[0];
  y=new double[0];
  order_a=0;
  order_b=0;
  a[0]=1;
  b[0]=c;
}

// LTISys::LTISys(const double* tb, int ob)
// {
//   int ib=ob;
//   while (ib>=0&&tb[ib]==0)
//     ib--;
// 
//   if (ib==-1)
//   {
//     a=new double[1];
//     b=new double[1];
//     x=new double[0];
//     y=new double[0];
//     order_a=0;
//     order_b=0;
//     a[0]=1;
//     b[0]=0;
//   }
//   else
//   {
//     order_b=ib;
//     order_a=0;
//     b=new double[ib+1];
//     a=new double[1];
//     x=new double[ib];
//     y=new double[0];
//     for (int i=0;i<=ib;i++)
//       b[i]=tb[i];
//     for(int i=0;i<ib;i++)
//       x[i]=0;
//     a[0]=1;
//   }
// }

LTISys::LTISys(const double* tb, int ob)
{
  int ib=ob;
  while (ib>=0&&tb[ib]==0)
    ib--;
  //numerator is zero
  if (ib==-1)
  {
    order_b=0;
    b=new double[1];
    x=0;
    b[0]=0;
  }
  else
  {
    order_b=ib;
    b=new double[ib+1];
    x=new double[ib];
    for (int i=0;i<=ib;i++)
      b[i]=tb[i];
    for(int i=0;i<ib;i++)
      x[i]=0;
  }
  order_a=0;
  a=new double[1];
  y=new double[0];
  a[0]=1;
}

LTISys::LTISys(const double* tb, int ob, const double* ta, int oa, double gain)
{

  int ia=oa;
  while (ia>=0&&ta[ia]==0)
    ia--;
  //denumerator is zero
  if (ia==-1)
  {
    order_a=0;
    a=new double[1];
    y=0;
    a[0]=0;
  }
  else
  {
    order_a=ia;
    a=new double[ia+1];
    y=new double[ia];
    for (int i=0;i<=ia;i++)
      a[i]=ta[i];
    for(int i=0;i<ia;i++)
      y[i]=0;
  }
  
  int ib=ob;
  while (ib>=0&&tb[ib]==0)
    ib--;
  //numerator is zero
  if (ib==-1)
  {
    order_b=0;
    b=new double[1];
    x=0;
    b[0]=0;
  }
  else
  {
    order_b=ib;
    b=new double[ib+1];
    x=new double[ib];
    for (int i=0;i<=ib;i++)
      b[i]=tb[i];
    for(int i=0;i<ib;i++)
      x[i]=0;
  }
  for(int i=0;i<=ib;i++)
    b[i]*=gain;
}

LTISys::LTISys(const LTISys &rhs)
{
  order_a=rhs.order_a;
  order_b=rhs.order_b;
  a=new double[order_a+1];
  b=new double[order_b+1];
  x=new double[order_b];
  y=new double[order_a];
  for (int i=0;i<=order_a;i++)
    a[i]=rhs.a[i];
  for (int i=0;i<=order_b;i++)
    b[i]=rhs.b[i];
  for(int i=0;i<order_a;i++)
    y[i]=0;
  for(int i=0;i<order_b;i++)
    x[i]=0;
}

LTISys & LTISys::operator=(const LTISys &rhs)
{
  if (this==&rhs)
    return *this;
  
  order_a=rhs.order_a;
  order_b=rhs.order_b;
  delete[] a;
  delete[] b;
  delete[] x;
  delete[] y;
  a=new double[order_a+1];
  b=new double[order_b+1];
  
  if(order_a==0)
    y=0;
  else
    y=new double[order_a];
  
  if(order_b==0)
    x=0;
  else
    x=new double[order_b];
  
  for (int i=0;i<=order_a;i++)
    a[i]=rhs.a[i];
  for (int i=0;i<=order_b;i++)
    b[i]=rhs.b[i];
  for(int i=0;i<order_a;i++)
    y[i]=rhs.y[i];
  for(int i=0;i<order_b;i++)
    x[i]=rhs.x[i];
  return *this;
}

const LTISys LTISys::num() const
{
  return LTISys(b,order_b);
}

const LTISys LTISys::den() const
{
  return LTISys(a,order_a);
}

const LTISys LTISys::operator*(const LTISys &other) const
{
  int oa=order_a+other.order_a;
  int ob=order_b+other.order_b;
  double *ta=new double[oa+1];
  double *tb=new double[ob+1];
  for (int i=0;i<oa+1;i++)
  {
    ta[i]=0;
    for (int j=0;j<=i&&j<=order_a;j++)
    {
      if ((i-j)<=other.order_a)
        ta[i]+=a[j]*other.a[i-j];
    }
    //cout<<result.a[i]<<endl;
  }
  for (int i=0;i<ob+1;i++)
  {
    tb[i]=0;
    for (int j=0;j<=i&&j<=order_b;j++)
    {
      if ((i-j)<=other.order_b)
        tb[i]+=b[j]*other.b[i-j];
    }
  }
  LTISys result(tb,ob,ta,oa);
  delete[] ta;
  delete[] tb;
  return result;
}

// const LTISys LTISys::operator*(const LTISys &other) const
// {
//   LTISys result(1);
//   result.order_a=order_a+other.order_a;
//   //cout<<order_a<<' '<<other.order_a<<' '<<result.order_a<<endl;
//   result.order_b=order_b+other.order_b;
//   delete[] result.a;
//   delete[] result.b;
//   result.a=new double[result.order_a+1];
//   result.b=new double[result.order_b+1];
//   for (int i=0;i<result.order_a+1;i++)
//   {
//     result.a[i]=0;
//     for (int j=0;j<=i&&j<=order_a;j++)
//     {
//       if ((i-j)<=other.order_a)
//         result.a[i]+=a[j]*other.a[i-j];
//     }
//     //cout<<result.a[i]<<endl;
//   }
//   for (int i=0;i<result.order_b+1;i++)
//   {
//     result.b[i]=0;
//     for (int j=0;j<=i&&j<=order_b;j++)
//     {
//       if ((i-j)<=other.order_b)
//         result.b[i]+=b[j]*other.b[i-j];
//     }
//   }
//   return result;
// }

const LTISys LTISys::inv() const
{
  return LTISys(a,order_a,b,order_b);
}

const LTISys LTISys::operator/(const LTISys &other) const
{
  LTISys result=(*this)*other.inv();
  return result;
}

const LTISys LTISys::operator+(const LTISys &other) const
{
  LTISys ra=this->den()*other.den();
  LTISys rb1=this->num()*other.den();
  LTISys rb2=this->den()*other.num();

  int oa,ob;
  oa=ra.order_b;
  ob=(rb1.order_b>rb2.order_b)?rb1.order_b:rb2.order_b;

  double *ta=new double[oa+1];
  double *tb=new double[ob+1];

  for (int i=0;i<ob+1;i++)
  {
    tb[i]=0;
    if(i<=rb1.order_b)
      tb[i]+=rb1.b[i];
    if(i<=rb2.order_b)
      tb[i]+=rb2.b[i];
  }
  for (int i=0;i<oa+1;i++)
    ta[i]=ra.b[i];

  LTISys result(tb,ob,ta,oa);
  delete[] ta;
  delete[] tb;
  return result;
}

const LTISys LTISys::operator-() const
{
  LTISys result=*this;
  for (int i=0;i<order_b;i++)
    result.b[i]=-result.b[i];
  return result;
}

const LTISys LTISys::operator-(const LTISys &other) const
{
  return (*this)+(-other);
}


ostream& operator<<(ostream &os,const LTISys &obj)
  {
    for(int i=obj.order_b;i>=0;i--)
      os<<obj.b[i]<<',';
    os<<endl;
    for(int i=obj.order_a;i>=0;i--)
      os<<obj.a[i]<<',';
    return os;
  }
const LTISys LTISys::tustin(double ts) const
{
  double ssa[]={ts/2,ts/2};
  double ssb[]={-1,1};
  LTISys s(ssb,1,ssa,1);
  LTISys sa(ssa,1);
  LTISys sb(ssb,1);

  LTISys ra=LTISys(a[order_a]);
  for (int i=order_a-1;i>=0;i--)
    ra=s*ra+LTISys(a[i]);
  LTISys rb=LTISys(b[order_b]);
  for (int i=order_b-1;i>=0;i--)
    rb=s*rb+LTISys(b[i]);

  int n=order_a-order_b;
  if (n>0)
    for (int i=0;i<n;i++)
      rb=rb*sa;
  else
    for (int i=0;i<-n;i++)
      ra=ra*sa;
  LTISys r=rb.num()/ra.num();
  //LTISys *dbgr=&r;
  double c=r.a[r.order_a];
  for(int i=0;i<=r.order_a;i++)
    r.a[i]/=c;
  for(int i=0;i<=r.order_b;i++)
    r.b[i]/=c;
  return r;
}

void LTISys::initialize(double x0)
{
  for(int i=0;i<order_b;i++)
    x[i]=x0;
  double sa=0;
  double sb=0;
  for(int i=0;i<=order_a;i++)
    sa+=a[i];
  for(int i=0;i<=order_b;i++)
    sb+=b[i];
  double dc_gain=sb/sa;
  for(int i=0;i<order_a;i++)
    y[i]=x0*dc_gain;
}

double LTISys::filter(double xn)
{
  double yn=xn*b[order_b];
  //cout<<xn<<','<<b[1]<<',';
  for(int i=order_b-1;i>=0;i--)
    yn+=x[order_b-1-i]*b[i];
  //cout<<x[0]<<','<<b[0]<<',';
  for(int i=order_a-1;i>=0;i--)
    yn-=y[order_b-1-i]*a[i];
  yn/=a[order_b];
  //cout<<y[0]<<','<<a[0]<<','<<yn<<endl;
  for(int i=order_b-1;i>0;i--)
    x[i]=x[i-1];
  x[0]=xn;
  for(int i=order_a-1;i>0;i--)
    y[i]=y[i-1];
  y[0]=yn;
  
  return yn;
}