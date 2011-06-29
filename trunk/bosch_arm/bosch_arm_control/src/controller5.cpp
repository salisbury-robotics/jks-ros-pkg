/**This code is for a simple cartesian space controller
  *
**/

#include <sys/time.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <pthread.h>

#include <algorithm>
#include <iostream>
#include <sstream>
#include <vector>

#include "cc.h"
#include "control.h"
#include "daq.h"

#include "ros/ros.h"
#include <bosch_arm_control/TipState.h>
#include <bosch_arm_control/PointCmd.h>
#include <bosch_arm_control/Diagnostic.h>

#define TIMER_HZ 2000000 //DAQ card clock
#define CYCLE_HZ 1000 //Servo cycle rate
#define CYCLE_COUNTS (TIMER_HZ/CYCLE_HZ)
#define CNT2SEC (1/(float)TIMER_HZ)

using namespace std;

pthread_mutex_t g_mutex;
pthread_cond_t  g_cond;
static int quit = 0;
pthread_t servo;


void * servo_loop2 ( void *ptr );
vector<double> get_Joint_Pos_Actual ( void );



class Robot
{
private:
  double home_offsets[4];
  double ql[4];
  uint16_t time_now;
  uint16_t time_last; //for wait
  uint16_t time_now2;
  uint16_t time_last2;//for update computing v
public:
  double q[4];
  double v[4];
  double torque[4];
  double v_lims[4];
  double t_lims[4];
  //setup the board and initialize registers.
  void initialize()
  {
    setup626();
    time_now=S626_CounterReadLatch ( constants::board0,constants::cntr_chan );
    time_last = time_now;
    time_now2= time_now;
    time_last2= time_now;
    home_offsets[0]=read_encoder ( 0 ) *constants::cnt2mdeg;
    home_offsets[1]=read_encoder ( 1 ) *constants::cnt2mdeg;
    home_offsets[2]=read_encoder ( 3 ) *constants::cnt2mdeg;
    home_offsets[3]=read_encoder ( 2 ) *constants::cnt2mdeg;
    for ( int i=0;i<4;i++ )
    {
      q[i]=0;
      ql[i]=0;
      v[i]=0;
      t_lims[i]=0.4*constants::t_max;
    }      
    double vlim=constants::v_lim;
    v_lims[0]=vlim;
    v_lims[1]=2*vlim;
    v_lims[2]=2*vlim;
    v_lims[3]=2*vlim;
    zero_torques();

  }
  //convert robot time to common time.
  double convertToWallTime ( uint16_t now,uint16_t last )
  {
    uint16_t t = ( now - last );
    return ( double ) t*CNT2SEC;
  }
  //get the hardware time
  uint16_t getTime()
  {
    return S626_CounterReadLatch ( constants::board0,constants::cntr_chan );
  }
  vector<double> getJointPosition ( void )
  {
    double motors [] = {q[0],q[2],q[1],q[3]};
    double joints [] = {0.0,0.0,0.0,0.0};
    for ( int i = 0;i<4;i++ )
    {
      for ( int j = 0;j<4;j++ )
      {
        joints[i]+=constants::m2j[4*i+j]*motors[j];  // Convert motor positions to joint positions
      }
    }
    vector<double> Pos ( joints, joints + sizeof ( joints ) );
    return Pos;
  }

  vector<double> getMotorPosition ( vector<double> Pos )
  {
    double m [] = {0.0,0.0,0.0,0.0};
    for ( int i = 0;i<4;i++ )
    {
      for ( int j = 0;j<4;j++ )
      {
        m[i]+=constants::j2m[4*i+j]*Pos[j];  // Convert motor positions to joint positions
      }
    }
    double actuators[]={m[0],m[2],m[1],m[3]};
    vector<double> Pos2 ( actuators, actuators + sizeof ( actuators ) );
    return Pos2;

  }
  
  void safe_exit ( const char* msg )
  {
    close();
    ROS_ERROR ( msg );
    //pthread_kill(servo, SIGTERM);
    exit ( 1 );

  }
  void truncate ( double &x, double max )
  {
    if ( x>max )
      x=max;
    if ( x<-max )
      x=-max;
  }
  void enforce_safety()
  {
    for(int i=0;i<4;i++)
    {
      if ( fabs ( v[i] ) >v_lims[i] )
        safe_exit ( "overspeed" );
      if ( fabs ( torque[i] ) >5*constants::t_max )
        safe_exit ( "torque command over 5 times max achievable" );
      truncate ( torque[i],t_lims[i] );
    }
  }
  void update()
  {
    time_now2=S626_CounterReadLatch ( constants::board0,constants::cntr_chan );
    q[0] = -read_encoder ( 0 ) *constants::cnt2mdeg + home_offsets[0];
    q[1] = -read_encoder ( 1 ) *constants::cnt2mdeg + home_offsets[1];
    q[2] = read_encoder ( 3 ) *constants::cnt2mdeg - home_offsets[2];
    q[3] = read_encoder ( 2 ) *constants::cnt2mdeg - home_offsets[3];
    double dt=convertToWallTime ( time_now2,time_last2 );
    time_last2=time_now2;
    for ( int i=0;i<4;i++ )
    {
      v[i]= ( q[i]-ql[i] ) /dt;
      ql[i]=q[i];
    }
    enforce_safety();
    write_torque ( 0,torque[0] );
    write_torque ( 1,torque[1] );
    write_torque ( 2,torque[2] );
    write_torque ( 3,torque[3] );
  }
  void close()
  {
    zero_torques();
    S626_InterruptEnable ( constants::board0, FALSE );
    S626_CloseBoard ( constants::board0 );
    cout << "626 closed out." << endl;
  }
  void wait()
  {
    do 
    {
      time_now=getTime();
      printf("%d\t",time_now);
    }while ( ( uint16_t ) ( time_now - time_last ) < CYCLE_COUNTS );
    printf("***\n");
    //printf("%d\n",(uint16_t)(time_now-time_last));
    time_last = time_now;
  }
};

Robot* rob_ptr;

class SimpleCartesianController
{
private:
  double v[4];
  double dq[4];
  double torque[4];
  double q[4];
  Robot* rob;
  ros::NodeHandle n;
  ros::Publisher diagnostic_pub;
  ros::Publisher tip_state_pub;
  ros::Subscriber sigle_point_cmd_sub;
  bosch_arm_control::Diagnostic diag;
  bosch_arm_control::TipState js;

  void singlePtCmdCallBack ( const bosch_arm_control::PointCmd& msg )
  {
    vector<double> act=rob->getMotorPosition ( msg.coordinates );
    for ( int i=0;i<4;i++ )
      qd[i]=act[i];

  }
public:
  double Kp[4];
  double Kv[4];
  double qd[4];
  double e_lim;
  double cutoff;
  double loop_time;
  timespec ts;

  SimpleCartesianController ( Robot *ptr )
  {
    rob=ptr;
    diagnostic_pub =  n.advertise<bosch_arm_control::Diagnostic> ( "/diagnostics",100 );
    sigle_point_cmd_sub = n.subscribe ( "/cartesian_point_cmd",3, &SimpleCartesianController::singlePtCmdCallBack, this );
    tip_state_pub =  n.advertise<bosch_arm_control::TipState> ( "/tip_states",100 );
    cutoff = 1000;
    loop_time = 0.001;
    e_lim = constants::p_err_lim;

    
    for ( int i=0;i<4;i++ )
    {

      Kp[i]=0.013;
      Kv[i] = 0.0001;
      qd[i]=0;
      v[i]=0;
      dq[i]=0;
      q[i]=0;
      torque[i]=0;
      //ql[i]=0;
    }
  }
  void start()
  {
    rob->initialize();
  }


  void update()
  {
    rob->update();
    //t_now=rob->time_now;
    clock_gettime ( CLOCK_REALTIME, &ts );
    //double dt=rob->convertToWallTime ( t_now,t_last );
    //t_last=t_now;
    double lambda = exp ( -2*3.14*cutoff*loop_time );
    for ( int i=0;i<4;i++ )
      q[i]=rob->q[i];

    for ( int i=0;i<4;i++ )
    {
      dq[i]=qd[i]-q[i];
      if ( fabs ( dq[i] ) >e_lim )
        rob->safe_exit ( "position error too large" );
      float dq_max=1.0*rob->t_lims[i]/Kp[i];
      rob->truncate ( dq[i],dq_max );
      v[i]= rob->v[i] * ( 1-lambda ) + lambda*v[i];      
      torque[i]=Kp[i]*dq[i]-Kv[i]*v[i];      
    }

    for ( int i=0;i<4;i++ )
      rob->torque[i]=torque[i];

    logging();

  }
  void logging()
  {
    std::string log;
    std::stringstream out;
    for ( int i=0;i<4;i++ )
      out<<q[i]<<",";
    for ( int i=0;i<4;i++ )
      out<<qd[i]<<",";
    for ( int i=0;i<4;i++ )
      out<<v[i]<<",";
    for ( int i=0;i<4;i++ )
      out<<torque[i]<<",";
    out << ts.tv_sec << ','<< ts.tv_nsec;

    log = out.str();
    diag.data=log;
    diag.header.stamp.sec=ts.tv_sec;
    diag.header.stamp.nsec=ts.tv_nsec;
    diagnostic_pub.publish ( diag );
    vector<double> cur_pos_act = rob->getJointPosition();
    js.header.stamp.sec=ts.tv_sec;
    js.header.stamp.nsec=ts.tv_nsec;
    js.position=cur_pos_act;
    tip_state_pub.publish ( js );
  }
  void stop()
  {
    rob->close();
  }

};

SimpleCartesianController* ctr_ptr;

static int kbhit ( void )
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr ( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~ ( ICANON | ECHO );
  tcsetattr ( STDIN_FILENO, TCSANOW, &newt );
  oldf = fcntl ( STDIN_FILENO, F_GETFL, 0 );
  fcntl ( STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK );

  ch = getchar();

  tcsetattr ( STDIN_FILENO, TCSANOW, &oldt );
  fcntl ( STDIN_FILENO, F_SETFL, oldf );

  if ( ch != EOF )
  {
    ungetc ( ch, stdin );
    return 1;
  }

  return 0;
}


int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "bosch_arm_node" );
  ros::NodeHandle n;
  rob_ptr=new Robot();
  ctr_ptr=new SimpleCartesianController ( rob_ptr );
  //diagnostic_pub =  n.advertise<bosch_arm_control::Diagnostic> ( "/diagnostics",100 );
  //ros::Subscriber sigle_point_cmd_sub = n.subscribe ( "/single_point_cmd",3, &SimplePDController::singlePtCmdCallBack,ctr_ptr );
  int result;

  // start servo thread
  pthread_attr_t  attributes;

  pthread_attr_init ( &attributes );
  pthread_attr_setdetachstate ( &attributes, PTHREAD_CREATE_JOINABLE );

  pthread_mutex_init ( &g_mutex, NULL );
  pthread_cond_init ( &g_cond, NULL );
  pthread_mutex_lock ( &g_mutex );
  char c = 0;
  result = pthread_create ( &servo, &attributes, servo_loop2, NULL );
  if ( result == 0 ) cout << "Servo thread started." << endl;
  while ( 1 )
  {
    pthread_cond_wait ( &g_cond, &g_mutex );
    printf("+++%d+++",rob_ptr->getTime());
    ros::spinOnce();
    if ( kbhit() )
    {// keyboard command
      c = getchar();
      rewind ( stdout );
      ftruncate ( 1,0 );
    }
    printf("%d+++\n",rob_ptr->getTime());
    if ( c == 'q' || c=='Q' ) break;
  }
  //pthread_join ( servo, NULL );
  rob_ptr->close();
  return 0;
}

void * servo_loop2 ( void *ptr )
{

  ctr_ptr->start();
//   cout<<"Position the arm at the zero position, then hit Enter to continue"<<endl;
//   char ch;
//   cin.get ( ch );
//   cin.clear();
  while ( 1 )
  {
    
    pthread_cond_signal ( &g_cond );    
    pthread_mutex_unlock ( &g_mutex );
    //printf("###%d###\n",rob_ptr->getTime());
    rob_ptr->wait();
    if ( quit )
      break;
    pthread_mutex_lock ( &g_mutex );
    ctr_ptr->update();
    //ctr_ptr->logging();
  }
  ctr_ptr->stop();
}


// vector<double> get_Joint_Pos ( void )
// {
//   double motors [] = {q1d,q3d,q2d,q4d};
//   double joints [] = {0.0,0.0,0.0,0.0};
//   for ( int i = 0;i<4;i++ )
//   {
//     for ( int j = 0;j<4;j++ )
//     {
//       joints[i]+=constants::m2j[4*i+j]*motors[j];  // Convert motor positions to joint positions
//     }
//   }
//   vector<double> Pos ( joints, joints + sizeof ( joints ) );
//   return Pos;
// }
//


// vector<double> get_Joint_Vel(void){
//     double motors [] = {v1,v3,v2,v4};
//     double joints [] = {0.0,0.0,0.0,0.0};
//     for(int i = 0;i<4;i++){
//         for (int j = 0;j<4;j++){
//             joints[i]+=constants::m2j[4*i+j]*motors[j];  // Convert motor positions to joint positions
//         }
//     }
//     vector<double> Pos(joints, joints + sizeof(joints));
//     return Pos;
// }



// vector<double> get_Joint_Vel ( void )
// {
//   double array[] = { v1, v2 - v3, v2 + v3, v4 };
//   vector<double> Vel ( array, array + sizeof ( array ) );
//
//   return Vel;
//
// }

