#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include "bosch_arm.h"
#include "simple_cartesian_controller.h"
#include "simple_joint_controller.h"
#include "trajectory_controller3.h"
using namespace std;
pthread_mutex_t g_mutex;
pthread_cond_t  g_cond;
static int quit = 0;
pthread_t servo;
BoschArm *rob_ptr;
TrajectoryController *ctr_ptr;
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
int main(int argc, char** argv)
{
  ros::init(argc, argv, "bosch_arm_node");
  //ros::NodeHandle n;
  rob_ptr=new BoschArm();
  //ctr_ptr=new SimpleCartesianController(rob_ptr);
  //ctr_ptr=new SimpleJointController(rob_ptr);
  ctr_ptr=new TrajectoryController(rob_ptr);
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
    switch(c)
    {
      case 'a':
      {
        ctr_ptr->Kp[3]+=0.01;
        cout<<"Kp[3]:"<<ctr_ptr->Kp[3]<<endl;
        break;
      }
      case 's':
      {
        ctr_ptr->Kp[3]-=0.01;
        cout<<"Kp[3]:"<<ctr_ptr->Kp[3]<<endl;
        break;
      }
      case 'g':
      {
        ctr_ptr->ctr_mode=2;
        cout<<"Gravity compensation mode"<<endl;
        break;
      }
      case 'p':
      {
        ctr_ptr->ctr_mode=1;
        cout<<"PD control with gravity compensation"<<endl;
        break;
      }
      case 'n':
      {
        ctr_ptr->ctr_mode=3;
        cout<<"relax"<<endl;
        break;
      }
    }
    c=0;
  }
  //pthread_join ( servo, NULL );
  rob_ptr->close();
  return 0;
}

