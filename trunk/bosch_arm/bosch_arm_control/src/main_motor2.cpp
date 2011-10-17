#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include "bosch_arm.h"
//#include "simple_cartesian_controller.h"
//#include "simple_joint_controller.h"
#include "motor_controller2.h"
//#include "cartesian_controller2.h"
using namespace std;
pthread_mutex_t g_mutex;
pthread_cond_t  g_cond;
static int quit = 0;
pthread_t servo;
BoschArm *rob_ptr;
TrajectoryController *ctr_ptr;
//CartesianController *ctr_ptr;
//SimpleCartesianController *ctr_ptr;
//SimpleJointController *ctr_ptr;
int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

void * servo_loop2(void *ptr)
{

  ctr_ptr->start();
//   cout<<"Position the arm at the zero position, then hit Enter to continue"<<endl;
//   char ch;
//   cin.get ( ch );
//   cin.clear();
  while (1)
  {

    pthread_cond_signal(&g_cond);
    pthread_mutex_unlock(&g_mutex);
    //printf("###%d###\n",rob_ptr->getTime());
    rob_ptr->wait();
    if (quit)
      break;
    pthread_mutex_lock(&g_mutex);
    ctr_ptr->update();
    //ctr_ptr->logging();
  }
  ctr_ptr->stop();
}
//enum {CALIBRATION, SERVO,LOADTRACE,SEMIAUTO};
enum {KP_VAR,KV_VAR,TUNE_STEP,V_STEP_TIME,V_STEP_SIZE,FILTER_CUT_OFF,CHIRP_STEP,FILTER_PARA};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "bosch_arm_node");
  //ros::NodeHandle n;
  rob_ptr=new BoschArm();
  //ctr_ptr=new SimpleCartesianController(rob_ptr);
  //ctr_ptr=new SimpleJointController(rob_ptr);
  ctr_ptr=new TrajectoryController(rob_ptr);
  //ctr_ptr=new CartesianController(rob_ptr);
  //diagnostic_pub =  n.advertise<bosch_arm_control::Diagnostic> ( "/diagnostics",100 );
  //ros::Subscriber sigle_point_cmd_sub = n.subscribe ( "/single_point_cmd",3, &SimplePDController::singlePtCmdCallBack,ctr_ptr );
  int result;

  // start servo thread
  pthread_attr_t  attributes;

  pthread_attr_init(&attributes);
  pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_JOINABLE);

  pthread_mutex_init(&g_mutex, NULL);
  pthread_cond_init(&g_cond, NULL);
  pthread_mutex_lock(&g_mutex);
  char c = 0;
  result = pthread_create(&servo, &attributes, servo_loop2, NULL);
  if (result == 0) cout << "Servo thread started." << endl;
  int joint=4;
  int var=KP_VAR;
  double fstep=0.01;
  double cutoff=400;
  double lambda=0;
  char space=0;
  int npara=6;
  while (1)
  {
    pthread_cond_wait(&g_cond, &g_mutex);
    //printf("+++%d+++",rob_ptr->getTime());
    ros::spinOnce();
    if (kbhit())
    {// keyboard command
      c = getchar();
      rewind(stdout);
      ftruncate(1,0);
    }
    //printf("%d+++\n",rob_ptr->getTime());
    if (c == 'q' || c=='Q') break;
    switch (c)
    {
    case '4':
    {
      joint=4;
      if (space=='j')
        cout<<"choose joint "<<joint<<endl;
      else if (space=='m')
        cout<<"choose motor "<<joint<<endl;
      break;
    }
    case '1':
    {
      joint=1;
      if (space=='j')
        cout<<"choose joint "<<joint<<endl;
      else if (space=='m')
        cout<<"choose motor "<<joint<<endl;
      break;
    }
    case '2':
    {
      joint=2;
      if (space=='j')
        cout<<"choose joint "<<joint<<endl;
      else if (space=='m')
        cout<<"choose motor "<<joint<<endl;
      break;
    }
    case '3':
    {
      joint=3;
      if (space=='j')
        cout<<"choose joint "<<joint<<endl;
      else if (space=='m')
        cout<<"choose motor "<<joint<<endl;
      break;
    }
    case '=':
    {
      if (var==KP_VAR)
      {
        if (space=='j')
        {
          ctr_ptr->Kqp[joint-1]+=fstep;
          cout<<"Kqp["<<joint<<"]:"<<ctr_ptr->Kqp[joint-1]<<endl;
        }
        else if (space=='m')
        {
          ctr_ptr->Kmp[joint-1]+=fstep;
          cout<<"Kmp["<<joint<<"]:"<<ctr_ptr->Kmp[joint-1]<<endl;
        }

      }
      else if (var==KV_VAR)
      {
        if (space=='j')
        {
          ctr_ptr->Kqv[joint-1]+=fstep;
          cout<<"Kqv["<<joint<<"]:"<<ctr_ptr->Kqv[joint-1]<<endl;
        }
        else if (space=='m')
        {
          ctr_ptr->Kmv[joint-1]+=fstep;
          cout<<"Kmv["<<joint<<"]:"<<ctr_ptr->Kmv[joint-1]<<endl;
        }
      }
      else if (var==V_STEP_SIZE)
      {
        if (space=='j')
        {
          ctr_ptr->qvstep[joint-1]+=fstep;
          cout<<"qvstep["<<joint<<"]:"<<ctr_ptr->qvstep[joint-1]<<endl;
        }
        else if (space=='m')
        {
          ctr_ptr->mvstep[joint-1]+=fstep;
          cout<<"mvstep["<<joint<<"]:"<<ctr_ptr->mvstep[joint-1]<<endl;
        }
      }
      else if (var==V_STEP_TIME)
      {
        if (space=='j')
        {
          ctr_ptr->qvtime[joint-1]=(int)(ctr_ptr->qvtime[joint-1]+fstep);
          cout<<"qvtime["<<joint<<"]:"<<ctr_ptr->qvtime[joint-1]<<endl;
        }
        else if (space=='m')
        {
          ctr_ptr->mvtime[joint-1]=(int)(ctr_ptr->mvtime[joint-1]+fstep);
          cout<<"mvtime["<<joint<<"]:"<<ctr_ptr->mvtime[joint-1]<<endl;
        }
      }
      else if (var==FILTER_CUT_OFF)
      {
        cutoff+=fstep;
        lambda = exp(-2*3.14*cutoff*0.001);
        rob_ptr->b[0]=1-lambda;
        rob_ptr->a[1]=-lambda;
        cout<<"cutoff: "<<cutoff<<"\tlambda: "<<lambda<<endl;
      }
      else if (var==CHIRP_STEP)
      {
        ctr_ptr->chirp_step+=fstep;
        cout<<"chirp step: "<<ctr_ptr->chirp_step<<endl;
      }
      else if (var==FILTER_PARA)
      {
        ctr_ptr->fpara[joint-1][npara]+=fstep;
        cout<<"filter para "<<npara<<':';
        cout<<ctr_ptr->fpara[joint-1][npara]<<endl;
        ctr_ptr->updateFilter(joint-1);
      }
      break;
    }
    case '-':
    {
      if (var==KP_VAR)
      {
        if (space=='j')
        {
          ctr_ptr->Kqp[joint-1]-=fstep;
          cout<<"Kqp["<<joint<<"]:"<<ctr_ptr->Kqp[joint-1]<<endl;
        }
        else if (space=='m')
        {
          ctr_ptr->Kmp[joint-1]-=fstep;
          cout<<"Kmp["<<joint<<"]:"<<ctr_ptr->Kmp[joint-1]<<endl;
        }

      }
      else if (var==KV_VAR)
      {
        if (space=='j')
        {
          ctr_ptr->Kqv[joint-1]-=fstep;
          cout<<"Kqv["<<joint<<"]:"<<ctr_ptr->Kqv[joint-1]<<endl;
        }
        else if (space=='m')
        {
          ctr_ptr->Kmv[joint-1]-=fstep;
          cout<<"Kmv["<<joint<<"]:"<<ctr_ptr->Kmv[joint-1]<<endl;
        }
      }
      else if (var==V_STEP_SIZE)
      {
        if (space=='j')
        {
          ctr_ptr->qvstep[joint-1]-=fstep;
          cout<<"qvstep["<<joint<<"]:"<<ctr_ptr->qvstep[joint-1]<<endl;
        }
        else if (space=='m')
        {
          ctr_ptr->mvstep[joint-1]-=fstep;
          cout<<"mvstep["<<joint<<"]:"<<ctr_ptr->mvstep[joint-1]<<endl;
        }
      }
      else if (var==V_STEP_TIME)
      {
        if (space=='j')
        {
          ctr_ptr->qvtime[joint-1]=(int)(ctr_ptr->qvtime[joint-1]-fstep);
          cout<<"qvtime["<<joint<<"]:"<<ctr_ptr->qvtime[joint-1]<<endl;
        }
        else if (space=='m')
        {
          ctr_ptr->mvtime[joint-1]=(int)(ctr_ptr->mvtime[joint-1]-fstep);
          cout<<"mvtime["<<joint<<"]:"<<ctr_ptr->mvtime[joint-1]<<endl;
        }
      }
      else if (var==FILTER_CUT_OFF)
      {
        cutoff-=fstep;
        lambda = exp(-2*3.14*cutoff*0.001);
        rob_ptr->b[0]=1-lambda;
        rob_ptr->a[1]=-lambda;
        cout<<"cutoff: "<<cutoff<<"\tlambda: "<<lambda<<endl;
      }
      else if (var==CHIRP_STEP)
      {
        ctr_ptr->chirp_step-=fstep;
        cout<<"chirp step: "<<ctr_ptr->chirp_step<<endl;
      }
      else if (var==FILTER_PARA)
      {
        ctr_ptr->fpara[joint-1][npara]-=fstep;
        cout<<"filter para "<<npara<<':';
      cout<<ctr_ptr->fpara[joint-1][npara]<<endl;
      ctr_ptr->updateFilter(joint-1);
      }
      break;
    }

    case 'p':
    {
      var=KP_VAR;
      if (space=='j')
      {
        fstep=0.1;
        cout<<"select Kqp["<<joint<<"]:"<<ctr_ptr->Kqp[joint-1]<<endl;
      }
      else if (space=='m')
      {
        fstep=0.1;
        cout<<"select Kmp["<<joint<<"]:"<<ctr_ptr->Kmp[joint-1]<<endl;
      }
      break;
    }
    case 'v':
    {
      var=KV_VAR;
      if (space=='j')
      {
        fstep=0.01;
        cout<<"select Kqv["<<joint<<"]:"<<ctr_ptr->Kqv[joint-1]<<endl;
      }
      else if (space=='m')
      {
        fstep=0.01;
        cout<<"select Kmv["<<joint<<"]:"<<ctr_ptr->Kmv[joint-1]<<endl;
      }
      break;
    }
    case 'w':
    {
      fstep/=10;
      cout<<"tuning step: "<<fstep<<endl;
      break;
    }
    case 'e':
    {
      fstep*=10;
      cout<<"tuning step: "<<fstep<<endl;
      break;
    }
    case 'c':
    {
      var=FILTER_CUT_OFF;
      fstep=100;
      cout<<"select cutoff: "<<cutoff<<"\tlambda: "<<lambda<<endl;
      break;
    }
    case 'n':
    {
      var=FILTER_PARA;
      npara=(npara+1)%7;
      if(npara%2==0)
        fstep=1;
      else
        fstep=0.1;
      cout<<"select filter para "<<npara<<':';
      cout<<ctr_ptr->fpara[joint-1][npara]<<endl;
      break;
    }
    case 'b':
    {
      var=CHIRP_STEP;
      fstep=1;
      cout<<"select chirp step: "<<ctr_ptr->chirp_step<<endl;
      break;
    }
    case 't':
    {
      var=V_STEP_TIME;
      if (space=='j')
      {
        cout<<"select qvtime["<<joint<<"]:"<<ctr_ptr->qvtime[joint-1]<<"(msec)"<<endl;
        fstep=100;
      }
      else if (space=='m')
      {
        cout<<"select mvtime["<<joint<<"]:"<<ctr_ptr->mvtime[joint-1]<<"(msec)"<<endl;
        fstep=100;
      }
      break;
    }
    case 'z':
    {
      var=V_STEP_SIZE;
      if (space=='j')
      {
        cout<<"select qvstep["<<joint<<"]:"<<ctr_ptr->qvstep[joint-1]<<"(rad/msec)"<<endl;
        fstep=0.01;
      }
      else if (space=='m')
      {
        cout<<"select mvstep["<<joint<<"]:"<<ctr_ptr->mvstep[joint-1]<<"(rad/msec)"<<endl;
        fstep=0.01;
      }
      break;
    }
    case 's':
    {
      if (space=='j')
      {
        double step=ctr_ptr->qtmax[joint-1]/ctr_ptr->Kqp[joint-1];
        ctr_ptr->qstep(joint-1,step);
        cout<<"joint "<<joint<<" step up "<<step<<endl;
      }
      else if (space=='m')
      {
        double step=ctr_ptr->mtmax[joint-1]/ctr_ptr->Kmp[joint-1];
        ctr_ptr->mstep(joint-1,step);
        cout<<"motor "<<joint<<" step up "<<step<<endl;
      }
      break;
    }
    case 'd':
    {
      if (space=='j')
      {
        double step=ctr_ptr->qtmax[joint-1]/ctr_ptr->Kqp[joint-1];
        ctr_ptr->qstep(joint-1,-step);
        cout<<"joint "<<joint<<" step down "<<step<<endl;
      }
      else if (space=='m')
      {
        double step=ctr_ptr->mtmax[joint-1]/ctr_ptr->Kmp[joint-1];
        ctr_ptr->mstep(joint-1,-step);
        cout<<"motor "<<joint<<" step down "<<step<<endl;
      }
      break;
    }
//     case 'i':
//     {
//       ctr_ptr->jid=joint-1;
//       ctr_ptr->dovstep=true;
//       ctr_ptr->tvstep=0;
//       ctr_ptr->ctr_mode=4;
//       for (int i=0;i<4;i++)
//       {
//         ctr_ptr->md[i]=ctr_ptr->m[i];
//         ctr_ptr->mv[i]=0;
//         ctr_ptr->mvd[i]=0;
//       }
//       cout<<"start joint "<<joint<<" velocity step "<<endl;
//       break;
//
//     }
    case 'y':
    {
      ctr_ptr->jid=joint-1;
      ctr_ptr->do_chirp=true;
      ctr_ptr->mtvstep=0;
      ctr_ptr->filter[joint-1]->initialize(0);
      ctr_ptr->ctr_mode=TrajectoryController::SYSID;
      
      break;
    }
    case 'l':
    {
      ctr_ptr->close_loop=!ctr_ptr->close_loop;
      if(ctr_ptr->close_loop)
        cout<<"close loop"<<endl;
      else
        cout<<"open loop"<<endl;
      break;
    }
    case 'i':
    {
      if (space=='j')
      {
        ctr_ptr->jid=joint-1;
        ctr_ptr->do_qvstep=true;
        ctr_ptr->qtvstep=0;
        //ctr_ptr->ctr_mode=TrajectoryController::QD_J2M_KM;
        for (int i=0;i<4;i++)
        {
          ctr_ptr->qd[i]=ctr_ptr->q[i];
          ctr_ptr->qv[i]=0;
          ctr_ptr->qvd[i]=0;
        }
        cout<<"start joint "<<joint<<" velocity step "<<endl;
      }
      else if (space== 'm')
      {
        ctr_ptr->jid=joint-1;
        ctr_ptr->do_mvstep=true;
        ctr_ptr->mtvstep=0;
        //ctr_ptr->ctr_mode=TrajectoryController::MD_KM;
        for (int i=0;i<4;i++)
        {
          ctr_ptr->md[i]=ctr_ptr->m[i];
          ctr_ptr->mv[i]=0;
          ctr_ptr->mvd[i]=0;
        }
        cout<<"start motor "<<joint<<" velocity step "<<endl;
      }
      break;

    }
    
    
    
    case 'u':
    {
      if (space=='j')
      {
        
        //ctr_ptr->ctr_mode=TrajectoryController::QD_J2M_KM;
        for (int i=0;i<4;i++)
        {
          ctr_ptr->qd[i]=ctr_ptr->q[i];
          ctr_ptr->qv[i]=0;
          ctr_ptr->qvd[i]=0;
        }
        ctr_ptr->set_position_ref();
        ctr_ptr->jid=joint-1;
        ctr_ptr->do_qsin=true;
        ctr_ptr->qtvstep=0;
        cout<<"start joint "<<joint<<" velocity step "<<endl;
      }
      else if (space== 'm')
      {
        
        //ctr_ptr->ctr_mode=TrajectoryController::MD_KM;
        for (int i=0;i<4;i++)
        {
          ctr_ptr->md[i]=ctr_ptr->m[i];
          ctr_ptr->mv[i]=0;
          ctr_ptr->mvd[i]=0;
        }
        ctr_ptr->set_position_ref();
        ctr_ptr->jid=joint-1;
        ctr_ptr->do_msin=true;
        ctr_ptr->mtvstep=0;
        cout<<"start motor "<<joint<<" sin wave "<<endl;
      }
      break;

    }
    
    case 'o':
    {
      if(ctr_ptr->do_chirp)
      {
        ctr_ptr->do_chirp=false;
        cout<<"stop motor "<<joint<<" chirp "<<endl;
      }
      if (space=='j')
      {
        if(ctr_ptr->do_qvstep)
        {
        ctr_ptr->do_qvstep=false;        
        cout<<"stop joint "<<joint<<" velocity step "<<endl;
        }
        else if(ctr_ptr->do_qsin)
        {
        ctr_ptr->do_qsin=false;
        cout<<"stop joint "<<joint<<" sin wave "<<endl;
        }
      }
      else if (space=='m')
      {
        if(ctr_ptr->do_mvstep)
        {
        ctr_ptr->do_mvstep=false;        
        cout<<"stop motor "<<joint<<" velocity step "<<endl;
        }
        else if(ctr_ptr->do_msin)
        {
        ctr_ptr->do_msin=false;
        cout<<"stop motor "<<joint<<" sin wave "<<endl;
        }
      }
      break;

    }
    
    case 'h':
    {
      cout<<"5-9,0: select control mode"<<endl;
      cout<<"\t5: gc_m2jT"<<endl;
      cout<<"\t6: md_Km"<<endl;
      cout<<"\t7: qd_Kq_m2jT"<<endl;
      cout<<"\t8: qd_j2m_Km"<<endl;
      cout<<"\t9: qd_Kq_gc_m2jT"<<endl;
      cout<<"\t0: relax"<<endl;
      cout<<"j: select joint space"<<endl;
      cout<<"m: select motor space"<<endl;
      cout<<"1-4: select joint/motor"<<endl;
      cout<<"p: tune Kp"<<endl;
      cout<<"v: tune Kv"<<endl;
      cout<<"t: tune vtime"<<endl;
      cout<<"z: tune vsize"<<endl; 
    }

    case '5':
    {
      ctr_ptr->ctr_mode=TrajectoryController::GC_M2JT;
      cout<<"Gravity compensation mode"<<endl;
      break;
    }
//     case 'l':
//     {
//       ctr_ptr->ctr_mode=1;
//       for (int i=0;i<4;i++)
//       {
//         ctr_ptr->qd[i]=ctr_ptr->q[i];
//         ctr_ptr->v[i]=0;
//         ctr_ptr->vd[i]=0;
//       }
//       cout<<"PD control with gravity compensation"<<endl;
//       break;
//     }
    case '[':
    {
      ctr_ptr->set_position_ref();
      cout<<"set position reference point."<<endl;
      break;
    }
    case '\\':
    {
      ctr_ptr->set_gc();
      cout<<"set gravity compensation"<<endl;
      break;
    }
    case ']':
    {
      ctr_ptr->clear_ref();
      cout<<"clear reference point."<<endl;
      break;
    }
    case 'j':
    {
      space='j';
      cout<<"Tune in joint space."<<endl;
      break;
    }
    case 'm':
    {
      space='m';
      cout<<"Tune in motor space."<<endl;
      break;
    }
    case '6':
    {
      ctr_ptr->ctr_mode=TrajectoryController::MD_KM;
      //space='m';
      for (int i=0;i<4;i++)
      {
        ctr_ptr->md[i]=ctr_ptr->m[i];
        ctr_ptr->mv[i]=0;
        ctr_ptr->mvd[i]=0;
      }
      cout<<"PD control in motor space"<<endl;
      break;
    }
    case '8':
    {
      ctr_ptr->ctr_mode=TrajectoryController::QD_J2M_KM;
      //space='m';
      for (int i=0;i<4;i++)
      {
        ctr_ptr->qd[i]=ctr_ptr->q[i];
        ctr_ptr->qv[i]=0;
        ctr_ptr->qvd[i]=0;
      }
      cout<<"PD control in motor space(joint command)"<<endl;
      break;
    }
    case '7':
    {
      ctr_ptr->ctr_mode=TrajectoryController::QD_KQ_M2JT;
      //space='j';
      for (int i=0;i<4;i++)
      {
        ctr_ptr->qd[i]=ctr_ptr->q[i];
        ctr_ptr->qv[i]=0;
        ctr_ptr->qvd[i]=0;
      }
      cout<<"PD control in joint space"<<endl;
      break;
    }
    case '9':
    {
      ctr_ptr->ctr_mode=TrajectoryController::QD_KQ_GC_M2JT;
     // space='j';
      for (int i=0;i<4;i++)
      {
        ctr_ptr->qd[i]=ctr_ptr->q[i];
        ctr_ptr->qv[i]=0;
        ctr_ptr->qvd[i]=0;
      }
      cout<<"PD control in joint space(with gravity compensation)"<<endl;
      break;
    }
    case '0':
    {
      ctr_ptr->ctr_mode=TrajectoryController::NOCONTROL;
      cout<<"relax"<<endl;
      break;
    }

//       case 'b':
//       {
//         rob_ptr->t=0;
//         rob_ptr->debug_filter=true;
//       }

    }
    c=0;
  }
  //pthread_join ( servo, NULL );
  rob_ptr->close();
  return 0;
}

