#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include "bosch_arm.h"
//#include "simple_cartesian_controller.h"
//#include "simple_joint_controller.h"
#include "trajectory_controller3.h"
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
enum {KP_VAR,KV_VAR,TUNE_STEP,V_STEP_TIME,V_STEP_SIZE,FILTER_CUT_OFF};
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
      cout<<"choose joint "<<joint<<endl;
      break;
    }
    case '1':
    {
      joint=1;
      cout<<"choose joint "<<joint<<endl;
      break;
    }
    case '2':
    {
      joint=2;
      cout<<"choose joint "<<joint<<endl;
      break;
    }
    case '3':
    {
      joint=3;
      cout<<"choose joint "<<joint<<endl;
      break;
    }
    case '=':
    {
      if (var==KP_VAR)
      {
        ctr_ptr->Kp[joint-1]+=fstep;
        cout<<"Kp["<<joint<<"]:"<<ctr_ptr->Kp[joint-1]<<endl;
      }
      else if (var==KV_VAR)
      {
        ctr_ptr->Kv[joint-1]+=fstep;
        cout<<"Kv["<<joint<<"]:"<<ctr_ptr->Kv[joint-1]<<endl;
      }
      else if (var==V_STEP_SIZE)
      {
        ctr_ptr->vstep[joint-1]+=fstep;
        cout<<"vstep["<<joint<<"]:"<<ctr_ptr->vstep[joint-1]<<endl;
      }
      else if (var==V_STEP_TIME)
      {
        ctr_ptr->vtime[joint-1]=(int)(ctr_ptr->vtime[joint-1]+fstep);
        cout<<"vtime["<<joint<<"]:"<<ctr_ptr->vtime[joint-1]<<endl;
      }
      else if (var==FILTER_CUT_OFF)
      {
        cutoff+=fstep;
        lambda = exp(-2*3.14*cutoff*0.001);
        rob_ptr->b[0]=1-lambda;
        rob_ptr->a[1]=-lambda;
        cout<<"cutoff: "<<cutoff<<"\tlambda: "<<lambda<<endl;
      }
//       else if (var==TUNE_STEP)
//       {
//         fstep*=10;
//         cout<<"tuning step: "<<fstep<<endl;
//       }
      break;
    }
    case '-':
    {
      if (var==KP_VAR)
      {
        ctr_ptr->Kp[joint-1]-=fstep;
        cout<<"Kp["<<joint<<"]:"<<ctr_ptr->Kp[joint-1]<<endl;
      }
      else if (var==KV_VAR)
      {
        ctr_ptr->Kv[joint-1]-=fstep;
        cout<<"Kv["<<joint<<"]:"<<ctr_ptr->Kv[joint-1]<<endl;
      }
      else if (var==V_STEP_SIZE)
      {
        ctr_ptr->vstep[joint-1]-=fstep;
        cout<<"vstep["<<joint<<"]:"<<ctr_ptr->vstep[joint-1]<<endl;
      }
      else if (var==V_STEP_TIME)
      {
        ctr_ptr->vtime[joint-1]=(int)(ctr_ptr->vtime[joint-1]-fstep);
        cout<<"vtime["<<joint<<"]:"<<ctr_ptr->vtime[joint-1]<<endl;
      }
      else if (var==FILTER_CUT_OFF)
      {
        cutoff-=fstep;
        double lambda = exp(-2*3.14*cutoff*0.001);
        rob_ptr->b[0]=1-lambda;
        rob_ptr->a[1]=-lambda;
        cout<<"cutoff: "<<cutoff<<"\tlambda: "<<lambda<<endl;
      }
//       else if (var==TUNE_STEP)
//       {
//         fstep/=10;
//         cout<<"tuning step: "<<fstep<<endl;
//       }
      break;
    }

    case 'p':
    {
      var=KP_VAR;
      fstep=1;
      cout<<"select Kp["<<joint<<"]:"<<ctr_ptr->Kp[joint-1]<<endl; 
      break;
    }
    case 'v':
    {
      var=KV_VAR;
      fstep=0.01;
      cout<<"select Kv["<<joint<<"]:"<<ctr_ptr->Kv[joint-1]<<endl;
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
    case 't':
    {
      var=V_STEP_TIME;
      cout<<"select vtime["<<joint<<"]:"<<ctr_ptr->vtime[joint-1]<<"(msec)"<<endl;
      fstep=100;
      break;
    }
    case 'z':
    {
      var=V_STEP_SIZE;
      cout<<"select vstep["<<joint<<"]:"<<ctr_ptr->vstep[joint-1]<<"(rad/msec)"<<endl;
      fstep=0.001;
      break;
    }
    case 's':
    {
      double step=ctr_ptr->tmax[joint-1]/ctr_ptr->Kp[joint-1];
      ctr_ptr->step(joint-1,step);
      cout<<"joint "<<joint<<" step up "<<step<<endl;
      break;
    }
    case 'd':
    {
      double step=ctr_ptr->tmax[joint-1]/ctr_ptr->Kp[joint-1];
      ctr_ptr->step(joint-1,-step);
      cout<<"joint "<<joint<<" step down "<<step<<endl;
      break;
    }
    case 'i':
    {
      ctr_ptr->jid=joint-1;
      ctr_ptr->dovstep=true;
      ctr_ptr->tvstep=0;
      ctr_ptr->ctr_mode=1;
      for (int i=0;i<4;i++)
      {
        ctr_ptr->qd[i]=ctr_ptr->q[i];
        ctr_ptr->v[i]=0;
        ctr_ptr->vd[i]=0;
      }
      cout<<"start joint "<<joint<<" velocity step "<<endl;
      break;

    }
    case 'o':
    {
      ctr_ptr->dovstep=false;
      cout<<"stop joint "<<joint<<" velocity step "<<endl;
      break;

    }

    case 'f':
    {
      ctr_ptr->ctr_mode=2;
      cout<<"Gravity compensation mode"<<endl;
      break;
    }
    case 'l':
    {
      ctr_ptr->ctr_mode=1;
      for (int i=0;i<4;i++)
      {
        ctr_ptr->qd[i]=ctr_ptr->q[i];
        ctr_ptr->v[i]=0;
        ctr_ptr->vd[i]=0;
      }
      cout<<"PD control with gravity compensation"<<endl;
      break;
    }
    case 'r':
    {
      ctr_ptr->ctr_mode=3;
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

