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

const double r1 = 0.062811565304088; // joint 1, turns pers motor turn
  const double r2 = 0.12524850894632;  // joint 2-3-4, turns per motor turn
  const double r3 = 15.9206349206349;  // joint 1 motor turns per joint turn
  const double r4 = 7.98412698412698;  // joint 2-3-4, motor turns per joint turn
  const double m2j[16] = {            // Motor to joint position conversion matrix
       r1, 0.0, 0.0, 0.0,
      -r1, 0.0,  r2, 0.0,
      0.0, r2,  -r2, 0.0,
      0.0, 0.0, 0.0,  r2
  };
  const double j2m [16] = {            // Joint to motor position conversion matrix
       r3, 0.0, 0.0, 0.0,
       r4,  r4,  r4, 0.0,
       r4,  r4, 0.0, 0.0,
      0.0, 0.0, 0.0,  r4
  };
const double L0=0.27;
const double L3=0.50;
const double L4=0.48;
const double rad_per_count=1.25663706e-3;
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
  double q[4];//actuator position
  double v[4];//actuator speed
  double torque[4];//actuator torque
  double v_lims[4];//speed limit
  double t_lims[4];//torque limit
  double dt;
  //setup the board and initialize registers.
  void initialize()
  {
    setup626();
    time_now=S626_CounterReadLatch ( constants::board0,constants::cntr_chan );
    time_last = time_now;
    time_now2= time_now;
    time_last2= time_now;
    home_offsets[0]=read_encoder ( 0 ) *rad_per_count;
    home_offsets[1]=read_encoder ( 1 ) *rad_per_count;
    home_offsets[2]=read_encoder ( 3 ) *rad_per_count;
    home_offsets[3]=read_encoder ( 2 ) *rad_per_count;
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
  
  void motor2JointPosition(const double* motors, double* joints)
  {
    for(int i=0;i<4;i++)
    {
      joints[i]=0;
      for(int j=0;j<4;j++)
        joints[i]+=m2j[4*i+j]*motors[j];
    }
  }
  
  void joint2TipPosition(const double* joints, double* tip)
  {
    double q1=joints[0];
    double q2=joints[1];
    double q3=joints[2];
    double q4=joints[3];
    tip[0]= - L4*(sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) + cos(q2)*cos(q4)*sin(q1)) - L3*cos(q2)*sin(q1);
      tip[1]=  - L4*(cos(q4)*sin(q2) + cos(q2)*sin(q3)*sin(q4)) - L3*sin(q2);
      tip[2]=L0 - L4*(sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - cos(q1)*cos(q2)*cos(q4)) + L3*cos(q1)*cos(q2);
  }

  void getJacobianJoint2Tip(const double* joints, double* jacobian)
  {
    double q1=joints[0];
    double q2=joints[1];
    double q3=joints[2];
    double q4=joints[3];
    jacobian[0]=L4*(sin(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) - cos(q1)*cos(q2)*cos(q4)) - L3*cos(q1)*cos(q2);
    jacobian[1]=L4*(cos(q4)*sin(q1)*sin(q2) + cos(q2)*sin(q1)*sin(q3)*sin(q4)) + L3*sin(q1)*sin(q2);
    jacobian[2]=L4*sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2));
    jacobian[3]=-L4*(cos(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) - cos(q2)*sin(q1)*sin(q4));
    jacobian[4]=0;
    jacobian[5]=- L4*(cos(q2)*cos(q4) - sin(q2)*sin(q3)*sin(q4)) - L3*cos(q2);
    jacobian[6]=-L4*cos(q2)*cos(q3)*sin(q4);
    jacobian[7]= L4*(sin(q2)*sin(q4) - cos(q2)*cos(q4)*sin(q3));
    jacobian[8]=-L4*(sin(q4)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)) + cos(q2)*cos(q4)*sin(q1)) - L3*cos(q2)*sin(q1);
    jacobian[9]=- L4*(cos(q1)*cos(q4)*sin(q2) + cos(q1)*cos(q2)*sin(q3)*sin(q4)) - L3*cos(q1)*sin(q2);
    jacobian[10]=L4*sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2));
    jacobian[11]=-L4*(cos(q4)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)) + cos(q1)*cos(q2)*sin(q4));
  }
  void joint2MotorPosition(const double* joints, double* motors)
  {
    for(int i=0;i<4;i++)
    {
      motors[i]=0;
      for(int j=0;j<4;j++)
        motors[i]+=j2m[4*i+j]*joints[j];
    }
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
    q[0] = -read_encoder ( 0 ) *rad_per_count + home_offsets[0];
    q[1] = -read_encoder ( 1 ) *rad_per_count + home_offsets[1];
    q[2] = read_encoder ( 3 ) *rad_per_count - home_offsets[2];
    q[3] = read_encoder ( 2 ) *rad_per_count - home_offsets[3];
    dt=convertToWallTime ( time_now2,time_last2 );
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
      //printf("%d\t",time_now);
    }while ( ( uint16_t ) ( time_now - time_last ) < CYCLE_COUNTS );
    //printf("***\n");
    //printf("%d\n",(uint16_t)(time_now-time_last));
    time_last = time_now;
  }
};

Robot* rob_ptr;

class SimpleCartesianController
{
private:
  double xl[3];//last tip position
  double dx[3];//tip position error
  double q[4];//joint position
  double tj[4];//joint effort
  double torque[4];//motor torque
  Robot* rob;
  ros::NodeHandle n;
  ros::Publisher diagnostic_pub;
  ros::Publisher tip_state_pub;
  ros::Subscriber sigle_point_cmd_sub;
  bosch_arm_control::Diagnostic diag;
  bosch_arm_control::TipState js;

  void singlePtCmdCallBack ( const bosch_arm_control::PointCmd& msg )
  {
    for ( int i=0;i<3;i++ )
      xd[i]=msg.coordinates[i];
  }
public:
  double x[3];
  double Kp[3];
  double Kv[3];
  double xd[3];
  double v[3];
  double f[3];
  timespec ts;

  SimpleCartesianController ( Robot *ptr )
  {
    rob=ptr;
    diagnostic_pub =  n.advertise<bosch_arm_control::Diagnostic> ( "/diagnostics",100 );
    sigle_point_cmd_sub = n.subscribe ( "/cartesian_point_cmd",3, &SimpleCartesianController::singlePtCmdCallBack, this );
    tip_state_pub =  n.advertise<bosch_arm_control::TipState> ( "/tip_states",100 );    
    for ( int i=0;i<3;i++ )
    {
      Kp[i]=10;
      Kv[i] = 0.1;
      xd[i]=0;
      v[i]=0;
      dx[i]=0;
      x[i]=0;
      f[i]=0;
    }
  }
  void start()
  {
    rob->initialize();
    rob->motor2JointPosition(rob->q,q);
    rob->joint2TipPosition(q,x);
    for(int i=0;i<3;i++)
    {
      xl[i]=x[i];
      xd[i]=x[i];
    }
  }
  
  

  void update()
  {
    rob->update();
    //t_now=rob->time_now;
    clock_gettime ( CLOCK_REALTIME, &ts );
    
    //save the last tip position
    for(int i=0;i<3;i++)
      xl[i]=x[i];
    rob->motor2JointPosition(rob->q,q);
    rob->joint2TipPosition(q,x);
    //PD control in cartesian space
    for(int i=0;i<3;i++)
    {
      
      dx[i]=xd[i]-x[i];
      v[i]=(x[i]-xl[i])/rob->dt;
      f[i]=Kp[i]*dx[i]-Kv[i]*v[i];
    }
    
    //compute the torque at motor space
    double jacobian[12];
    rob->getJacobianJoint2Tip(q,jacobian);
    //cout<<"jacobian transpose:"<<endl;
    for(int i=0;i<4;i++)
    {
      tj[i]=0;
      for(int j=0;j<3;j++)
      {
        tj[i]+=jacobian[4*j+i]*f[j];
        //cout<<jacobian[4*j+i]<<',';
      }
     // cout<<endl;
    }
    //cout<<"-------------------"<<endl;
    //cout<<"transmission transpose:"<<endl;
    for(int i=0;i<4;i++)
    {
      torque[i]=0;
      for(int j=0;j<4;j++)
      {
        torque[i]+=m2j[4*j+i]*tj[j];
        //cout<<m2j[4*j+i]<<',';
      }
      //cout<<endl;
    }
    //cout<<"-------------------"<<endl;
    //cout<<"m    :";
//     for ( int i=0;i<3;i++ )
//       cout<<f[i]<<',';
//     cout<<endl;
    for ( int i=0;i<4;i++ )
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
  void logging()
  {
    std::string log;
    std::stringstream out;
    for ( int i=0;i<3;i++ )
      out<<x[i]<<",";
    for ( int i=0;i<3;i++ )
      out<<xd[i]<<",";
    for ( int i=0;i<3;i++ )
      out<<v[i]<<",";
    for ( int i=0;i<3;i++ )
      out<<f[i]<<",";
    out << ts.tv_sec << ','<< ts.tv_nsec;

    log = out.str();
    diag.data=log;
    diag.header.stamp.sec=ts.tv_sec;
    diag.header.stamp.nsec=ts.tv_nsec;
    diagnostic_pub.publish ( diag );
    vector<double> cur_pos_act(x,x+3);
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
    //printf("+++%d+++",rob_ptr->getTime());
    ros::spinOnce();
    if ( kbhit() )
    {// keyboard command
      c = getchar();
      rewind ( stdout );
      ftruncate ( 1,0 );
    }
    //printf("%d+++\n",rob_ptr->getTime());
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

