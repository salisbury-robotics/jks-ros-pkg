//#include "daq.cpp"
#include <iostream>
#include <algorithm>
#include <pthread.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <vector>
#include <signal.h>

using namespace std;
#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>

#include "cc.h"
#include "control.h"
#include "daq.h"

using namespace std;

pthread_mutex_t g_mutex;
pthread_cond_t  g_cond;

const double k_period = 0.001;
const int k_samples = 2000;
double g_dt1[k_samples];
double g_dt2[k_samples];
void *run_busy(void *);

#include <stdlib.h>
#include <iostream>
#include <pthread.h>
#include <string>
#include <sstream>

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include <math.h>

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#define TIMER_HZ 2000000 //DAQ card clock
#define CYCLE_HZ 1000 //Servo cycle rate
#define CYCLE_COUNTS (TIMER_HZ/CYCLE_HZ)
#define CNT2SEC (1/(float)TIMER_HZ)
#define STOPJOGGING jogplus = false;jogminus = false

//#define TESTING

static void process_input(char c);
void * servo_loop(void *ptr);
void * servo_loop2(void *ptr);
void delta_p(int axis, double degrees);
void diep(char *s);
void diep(const char *s);
void err(char *s);
void err(const char *s);
void * robot_server(void *ptr);
void exit_cleanup(void);
void serve_request(string command);

static int kbhit(void);
static int printu(char * hostname, int port, char * data);
static void jog(void);

static double t_lim1 = 0.4*constants::t_max;
static double t_lim2 = 0.4*constants::t_max;
static double t_lim3 = 0.4*constants::t_max;
static double t_lim4 = 0.4*constants::t_max;

static bool t_wave1 = false;
static bool t_wave2 = false;
static bool t_wave3 = false;
static bool t_wave4 = false;

static double vstep1 = 0.15;
static double vstep2 = 0.1;
static double vstep3 = 0.025;
static double vstep4 = 0.06;//1.2;

//static double q1, q2, q3, q4 = 0.0;     // current position, in degrees
//static double q1l, q2l, q3l, q4l = 0.0; // last position
//static double q1d, q2d, q3d, q4d = 0.0; // desired position

/*
static double q1home = -1073.45;
static double q2home = -230.04;
static double q3home = 70.848;
static double q4home = -1064.52;
*/
static double home_offsets [4] = {0.0,0.0,0.0,0.0};

//old
static double q1home = 0;
static double q2home = 0;
static double q3home = 0;
static double q4home = 0;

static double q1, q1l, q1d = q1home;
static double q2, q2l, q2d = q2home;
static double q3, q3l, q3d = q3home;
static double q4, q4l, q4d = q4home;

static double v1, v2, v3, v4 = 0;     // deg/s
static double v1l, v2l, v3l, v4l = 0; // last velocity
static double v1d, v2d, v3d = 0; // desired velocity
static double v4d = 360; // desired velocity

static double acc1 = 300;             // deg/s^2
static double acc2 = 300;
static double acc3 = 300;
static double acc4 = 300;

static double Kp1 = 0.013; //0.025;
static double Kv1 = 0.0001; //0.0000091;
static double Kp2 = 0.013;
static double Kv2 = 0.0001; //0.0000091;
static double Kp3 = 0.013;
static double Kv3 = 0.0001; //0.0000091;
static double Kp4 = 0.013; //0.036;
static double Kv4 = 0.0001; //0.000011;

static bool joint_space = true;
static bool flail_around = false;
static double flail_start = 0.0;
static double lastpos1, lastpos2, lastpos3, lastpos4 = 0.0;

//static double q1f_start, q2f_start, q3f_start, q4f_start = 0.0;

static int quit = 0;
static int jog_step = 50;
static double lambda = 1.0;
static double cutoff = 1000;
static int axis = 0;
static bool jogplus = false;
static bool jogminus = false;

static double loop_time = 0.001;
static int newcmd = 0;
static char cmdbuf[512];

static vector<double> cur_pos;
static vector<double> cur_pos_act;

enum Mode { NONE, JOG, INCREMENT, KP, KV, FILTER };
static Mode mode = NONE;

pthread_t servo;
pthread_t robot;

#ifndef TESTING
int main(int argc, char** argv){
    static int cmd = newcmd;
    static bool streaming = false;
    if(!setup626()){
        // home, must do before starting servo loop
        cout<<"Position the arm at the zero position, then hit Enter to continue"<<endl;
        char ch;
        cin.get(ch);
        home_offsets[0] = read_encoder(0)*constants::cnt2mdeg;
        home_offsets[1] = read_encoder(1)*constants::cnt2mdeg;
        home_offsets[2] = read_encoder(2)*constants::cnt2mdeg;
        home_offsets[3] = read_encoder(3)*constants::cnt2mdeg;
        cin.clear();

        int result;

        // start servo thread
        pthread_attr_t  attributes;

        pthread_attr_init(&attributes);
        pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_JOINABLE);

        pthread_mutex_init(&g_mutex, NULL);
        pthread_cond_init(&g_cond, NULL);
        pthread_mutex_lock(&g_mutex);
        
        result = pthread_create(&servo, &attributes, servo_loop, NULL);
        if (result == 0) cout << "Servo thread started." << endl;

        // start UDP server
        pthread_attr_t  r_attributes;

        pthread_attr_init(&r_attributes);
        pthread_attr_setdetachstate(&r_attributes, PTHREAD_CREATE_JOINABLE);

        result = pthread_create(&robot, &r_attributes, robot_server, NULL);
        if (result == 0) cout << "Robot server thread started." << endl;

        char c = 0;
        zero_torques();

        // run
        while(1){
            pthread_cond_wait(&g_cond, &g_mutex);
            if(cmd != newcmd){ // udp server issued a new command
                cmd = newcmd;  // reset the flag
                string command(cmdbuf);
                if(cmdbuf[0] == '%'){  // enter streaming mode
                    streaming = true;
                    command.erase(0,1);
                }
                if(streaming){
                    if(cmdbuf[strlen(cmdbuf)] == ';'){
                        streaming = false; // If the last character is ';', leave streaming mode
                        command.erase(command.length()-1,1);
                    }
                    serve_request(command);
                }
                else c = cmdbuf[0];
            }
            else if(kbhit()){// keyboard command
                c = getchar();
                rewind(stdout);
                ftruncate(1,0);
            }
            if(c){// if we have any new command
                printf("%c\r\n",c);
                process_input(c);
                //cout << "Kp1: " << Kp1 << "," <<  " Kv1: " << Kv1 << "," << "Kp2: " << Kp2 << "," <<  " Kv2: " << Kv3 << "," <<"Kp4: " << Kp4 << "," <<  " Kv4: " << Kv4 << "," << " cutoff (Hz): " << cutoff<<endl;
            }
            if(c == 'q' || c=='Q')break;
            c=0;
        }
        // wait for servo thread to exit cleanly
        pthread_join(servo, NULL);
        exit_cleanup();
    }
}
#endif

static void process_input(char c){
  float Kp = 1.0;
  float Kv = 1.0;
  int dir = -1;
  switch(c){
  case '1':{
    axis = 1;
    STOPJOGGING;
    break;
  }
  case '2':{
    axis = 2;
    STOPJOGGING;
    break;
  }
  case '3':{
    axis = 3;
    STOPJOGGING;
    break;
  }
  case '4':{
    axis = 4;
    STOPJOGGING;
    break;
  }
  case '=':{
          dir = 1;
          // Fall through
      }
  case '-':{
          if(mode == INCREMENT){
              if(joint_space){
                  vector<double> pos = get_Joint_Pos();
                  pos[axis-1] += 0.5*dir;  // Move 1/2 degree
                  //cout<<q1<<','<<q2<<','<<q3<<','<<q4<<endl;
                  set_Joint_Pos(pos);
              }
              else{
                  if(axis==1){
                      jog_step = t_lim1/Kp1;
                      q1d+=dir*jog_step;
                  }
                  if(axis==2){
                      jog_step = t_lim2/Kp2;
                      q2d+=dir*jog_step;
                  }
                  if(axis==3){
                      jog_step = t_lim3/Kp3;
                      q3d+=dir*jog_step;
                  }
                  if(axis==4){
                      jog_step = t_lim4/Kp4;
                      q4d+=dir*jog_step;
                  }
              }
          }
          if(mode == JOG){
              if(jogplus||jogminus){
                  STOPJOGGING;
              }
              else{
                  if(dir>0){
                      jogplus = true;
                      jogminus = false;
                  }
                  else{
                      jogminus = true;
                      jogplus = false;
                  }
              }
          }
          break;
  }
  case '0':{
    STOPJOGGING;
    break;
  }
  case 'w':{
          mode = JOG;
    break;
  }
  case 's':{
          mode = INCREMENT;
    break;
  }
  /*case 'w':{
    jog_step = t_lim1/Kp1;
    q1d+=jog_step;
    if(joint_space){
      q2d+=jog_step/2;
      q3d+=jog_step/2;
    }
    break;
  }
  case 's':{
          jog_step = t_lim1/Kp1;
          q1d-=jog_step;
          if(joint_space){
              q2d-=jog_step/2;
              q3d-=jog_step/2;
          }
          break;
      }
  case 'e':{
    jog_step = t_lim2/Kp2;
    q3d+=jog_step;
    if(joint_space)q2d+=jog_step;
    break;
  }
  case 'd':{
    jog_step = t_lim2/Kp2;
    q3d-=jog_step;
    if(joint_space)q2d-=jog_step;
    break;
  }
  case 'r':{
    jog_step = t_lim3/Kp3;
    q2d+=jog_step;
    break;
  }
  case 'f':{
    jog_step = t_lim3/Kp3;
    q2d-=jog_step;
    break;
  }
  case 't':{
          //double array[] = { 90,90,90,0 };
          //vector<double> pos(array, array + sizeof(array));
          vector<double> pos = get_Joint_Pos();
          pos[3] += 3;
          set_Joint_Pos(pos);
          //jog_step = t_lim4/Kp4;
          //q4d+=jog_step;
          break;
      }
  case 'g':{
    jog_step = t_lim4/Kp4;
    q4d-=jog_step;
    break;
  }*/
  case 'p':{
    Kp =1.1;
    break;
  }
  case 'l':{
    Kp =1/1.1;
    break;
  }
  case 'o':{
    Kv =1.1;
    break;
  }
  case 'k':{
    Kv =1/1.1;
    break;
  }
  case 'i':{
    cutoff *= 1.1;
    break;
  }
  case 'j':{
    cutoff /= 1.1;
    break;
  }
  case 'x':{
    t_wave1 = !t_wave1;
    break;
  } 
  case 'c':{
    t_wave2 = !t_wave2;
    break;
  } 
  case 'v':{
    t_wave3 = !t_wave3;
    break;
  } 
  case 'b':{
    t_wave4 = !t_wave4;
    break;
  } 
  case 'a':{
    joint_space = !joint_space;
    break;
  }
  case 'z':{
    flail_around = !flail_around;
    if(flail_around){
        timespec fs;
        clock_gettime(CLOCK_REALTIME, &fs);
        flail_start =  fs.tv_sec + fs.tv_nsec * 0.000000001;
        lastpos1 = lastpos2 = lastpos3 = lastpos4 = 0.0;
    }
    break;
  }
  case 'Q'://fall through
  case 'q':{
    quit = 1;
    break;
  }
  default:
    cout<<"Key commands:"<<endl<<
      "q:\tquit"<<endl<<
      "1,2,3,4:\tSelect a motor/joint"<<endl<<
      "=:\tpositive"<<endl<<
      "-:\tnegative"<<endl<<
      "w:\tmove incrementally"<<endl<<
      "s:\tmove by jogging"<<endl<<
      "p:\tKp+"<<endl<<
      "l:\tKp-"<<endl<<
      "o:\tKv+"<<endl<<
      "k:\tKv-"<<endl<<
      "i:\tFilter+"<<endl<<
      "j:\tFilter-"<<endl<<
      "a:\tToggle motor/joint control"<<endl<<
      "z:\tToggle Flail around"<<endl<<
      "x:\ttoggle motor1 triangle wave"<<endl<<
      "c:\ttoggle motor2 triangle wave"<<endl<<
      "v:\ttoggle motor3 triangle wave"<<endl<<
      "b:\ttoggle motor4 triangle wave"<<endl;
  }

  switch(axis){
  case 1:{
    Kp1 *= Kp;
    Kv1 *= Kv;
    break;
  }
  case 2:{
    Kp2 *=Kp;
    Kv2 *=Kv;
    break;
  }
  case 3:{
    Kp3 *=Kp;
    Kv3 *=Kv;
    break;
  }
  case 4:{
    Kp4 *=Kp;
    Kv4 *=Kv;
    break;
  }
  default:
    ; // no motor
  }

  return;
}

void * servo_loop(void *ptr){
  static uint16_t tlast = 0;
  static int i = 0;
  //static int j = 0;
  //static double q1c, q2c, q3c, q4c = 0; // commanded position, takes desired position and filters through acceleration
  static double v1c, v2c, v3c, v4c = 0; // commanded velocity, limits velocity changes to appropriate acceleration

  static timespec tl;//,ts;
  clock_gettime(CLOCK_REALTIME, &tl); 
 
  while(1){
    uint16_t time;
    uint16_t t;

    pthread_cond_signal(&g_cond);
    pthread_mutex_unlock(&g_mutex);
    do time = S626_CounterReadLatch(constants::board0,constants::cntr_chan);
    while((uint16_t)(time - tlast) < CYCLE_COUNTS);

    if(quit)break;  // Don't move this outside the g_cond/g_mutex manipulations
    pthread_mutex_lock(&g_mutex);
    
    t = (time - tlast);
    float dt = (float)t*CNT2SEC;
    tlast = time;
    
    // Jogging
    if(jogplus|jogminus)jog();

    // Triangle wave
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    if(t_wave1 && ts.tv_sec%6 >= 3){
      q1d+=vstep1;
      if(joint_space){
	q2d+=vstep1/2;
	q3d+=vstep1/2;
      }
    }
    if(t_wave1 && ts.tv_sec%6 < 3){
      q1d-=vstep1;
      if(joint_space){
	q2d-=vstep1/2;
	q3d-=vstep1/2;
      }
    }
    if(t_wave2 && ts.tv_sec%8 >= 4){
      q3d+=vstep2;
      if(joint_space)q2d+=vstep2;
    }
    if(t_wave2 && ts.tv_sec%8 < 4){
      q3d-=vstep2;
      if(joint_space)q2d-=vstep2;
    }
    if(t_wave3 && ts.tv_sec%6 >= 3)q2d+=vstep3;
    if(t_wave3 && ts.tv_sec%6 < 3)q2d-=vstep3;

    if(t_wave4 && ts.tv_sec%2 >= 1)q4d+=vstep4;
    if(t_wave4 && ts.tv_sec%2 < 1)q4d-=vstep4;

    // read in positions, in motor degrees
    q1 = -read_encoder(0)*constants::cnt2mdeg + home_offsets[0];
    q2 = -read_encoder(1)*constants::cnt2mdeg + home_offsets[1];
    q3 = read_encoder(3)*constants::cnt2mdeg - home_offsets[3]; // <-- notice, out of order, need to fix wiring at some point
    q4 = read_encoder(2)*constants::cnt2mdeg - home_offsets[2];

    // bail if something is too wrong
    bool bail = false;

    // position error limit
    float dq1 = q1d-q1;
    float dq2 = q2d-q2;
    float dq3 = q3d-q3;
    float dq4 = q4d-q4;

    double e_lim = constants::p_err_lim;
    if(dq1 > e_lim || dq2 > e_lim || dq3 > e_lim || dq4 > e_lim)bail = true;
    if(-dq1 > e_lim || -dq2 > e_lim || -dq3 > e_lim || -dq4 > e_lim)bail = true;
    if(bail)err("position error too large");

    // position command limiting
    float maxq1 = 1.0*t_lim1/Kp1;
    if(dq1 > maxq1)dq1 = maxq1;
    if(dq1 < -maxq1)dq1 = -maxq1;

    float maxq2 = 1.0*t_lim2/Kp2;
    if(dq2 > maxq2)dq2 = maxq2;
    if(dq2 < -maxq2)dq2 = -maxq2;

    float maxq3 = 1.0*t_lim3/Kp3;
    if(dq3 > maxq3)dq3 = maxq3;
    if(dq3 < -maxq3)dq3 = -maxq3;

    float maxq4 = 1.0*t_lim4/Kp4;
    if(dq4 > maxq4)dq4 = maxq4;
    if(dq4 < -maxq4)dq4 = -maxq4;

    // set up velocity filter
    lambda = exp(-2*3.14*cutoff*loop_time);

    // calculate velocities
    v1 = ((q1-q1l)/dt)*(1-lambda) + lambda*v1;
    v2 = ((q2-q2l)/dt)*(1-lambda) + lambda*v2;
    v3 = ((q3-q3l)/dt)*(1-lambda) + lambda*v3;
    v4 = ((q4-q4l)/dt)*(1-lambda) + lambda*v4;

    // velocity limit
    double vlim = constants::v_lim;
    if(v1 > vlim || v2 > 2*vlim || v3 > 2*vlim || v4 > 2*vlim)bail = true;
    if(-v1 > vlim || -v2 > 2*vlim || -v3 > 2*vlim || -v4 > 2*vlim)bail = true;
    if(bail)err("overspeed");

    if(flail_around){
        float tf =  (ts.tv_sec + ts.tv_nsec * 0.000000001) - flail_start;

        float A1 = 32*16.667;
        float w1 = 0.4;
        float shift1 = 0.12;
        float As1 = -0.5/shift1;
        float ws1 = w1*shift1;

        double pos1 = A1*sin(w1*tf + As1*sin(ws1*tf + sin(ws1*tf)));
        double deltapos1 = pos1-lastpos1;
        delta_p(1,deltapos1);
        lastpos1 = pos1;

        float A2 = 15*7.98;
        float w2 = 0.55;
        float shift2 = 0.1;
        float As2 = -0.5/shift2;
        float ws2 = w2*shift2;

        double pos2 = A2*sin(w2*tf + As2*sin(ws2*tf + sin(ws2*tf)));
        double deltapos2 = pos2-lastpos2;
        delta_p(2,deltapos2);
        lastpos2 = pos2;

        float A3 =20*7.98;
        float w3 = 1.0;
        float shift3 = 0.09;
        float As3 = -0.45/shift3;
        float ws3 = w3*shift3;

        double pos3 = A3*sin(w3*tf + As3*sin(ws3*tf + sin(ws3*tf)));
        double deltapos3 = pos3-lastpos3;
        delta_p(3,deltapos3);
        lastpos3 = pos3;

        float A4 = 60*7.98;
        float w4 = 0.6;
        float shift4 = 0.13;
        float As4 = -0.55/shift4;
        float ws4 = w4*shift4;

        double pos4 = A4*sin(w4*tf + As4*sin(ws4*tf + sin(ws4*tf)));
        double deltapos4 = pos4-lastpos4;
        delta_p(4,deltapos4);
        lastpos4 = pos4;
        //cout<<tf<<","<<deltapos4<<endl;
    }

    /*where A, w are base amplitude (in whatever units - probably rad) and frequency (in rad/sec)
      and As, ws are the "phase-shift" amplitude and frequency (in rad and rad/sec).

      In particular, I used

      As = -0.5 / shift
      ws = w * shift

      where shift was the "how much to shift" parameter of ~0.1 (10%).*/

    //if(ts.tv_sec != tl.tv_sec){
    // cout<<dt<<","<<v4<<endl;
    //}
    //clock_gettime(CLOCK_REALTIME, &tl);
    //tl.tv_sec = ts.tv_sec;

    //if(i%1000 == 0)cout<<v4c<<","<<v4d<<","<<v4<<endl;

    // velocity command filtering (linear acceleration)
    float dv1 = v1d-v1c;
    float maxa1 = acc1/CYCLE_HZ;
    if(dv1 > maxa1)dv1 = maxa1;
    if(dv1 < -maxa1)dv1 = -maxa1;
    v1c += dv1;
    dv1 = v1c-v1;

    float dv2 = v2d-v2c;
    float maxa2 = acc2/CYCLE_HZ;
    if(dv2 > maxa2)dv2 = maxa2;
    if(dv2 < -maxa2)dv2 = -maxa2;
    v2c += dv2;
    dv2 = v2c-v2;

    float dv3 = v3d-v3c;
    float maxa3 = acc3/CYCLE_HZ;
    if(dv3 > maxa3)dv3 = maxa3;
    if(dv3 < -maxa3)dv3 = -maxa3;
    v3c += dv3;
    dv3 = v3c-v3;

    float dv4 = v4d-v4c;
    float maxa4 = acc4/CYCLE_HZ;
    if(dv4 > maxa4)dv4 = maxa4;
    if(dv4 < -maxa4)dv4 = -maxa4;
    v4c += dv4;
    dv4 = v4c-v4;

    // save last positions
    q1l = q1;
    q2l = q2;
    q3l = q3;
    q4l = q4;

    // save last velocities
    v1l = v1;
    v2l = v2;
    v3l = v3;
    v4l = v4;

    // calculate torque commands
    double torque1,torque2,torque3,torque4;
    torque1 = Kp1*dq1 - Kv1*v1;
    torque2 = Kp2*dq2 - Kv2*v2;
    torque3 = Kp3*dq3 - Kv3*v3;
    torque4 = Kp4*dq4 - Kv4*v4;

    // torque limit
    double t_lim = constants::t_max;
    if(torque1 > 5*t_lim || torque1 < 5*-t_lim)bail = true;
    if(torque2 > 5*t_lim || torque2 < 5*-t_lim)bail = true;
    if(torque3 > 5*t_lim || torque3 < 5*-t_lim)bail = true;
    if(torque4 > 5*t_lim || torque4 < 5*-t_lim)bail = true;
    if(bail)err("torque command over 5 times max achievable");

    // clip torque commands to max
    if(torque1 > t_lim1) torque1 = t_lim1;
    if(torque1 < -t_lim1) torque1 = -t_lim1;

    if(torque2 > t_lim2) torque2 = t_lim2;
    if(torque2 < -t_lim2) torque2 = -t_lim2;

    if(torque3 > t_lim3) torque3 = t_lim3;
    if(torque3 < -t_lim3) torque3 = -t_lim3;

    if(torque4 > t_lim4) torque4 = t_lim4;
    if(torque4 < -t_lim4) torque4 = -t_lim4;

    // tell DAQ card to output torques
    write_torque(0,torque1);
    write_torque(1,torque2);
    write_torque(2,torque3);
    write_torque(3,torque4);

    cur_pos = get_Joint_Pos();
    cur_pos_act = get_Joint_Pos_Actual();

    // datalogging
    std::string log;
    std::stringstream out;
    out << q1 << "," << q2 << "," << q3 << "," << q4 << ",";     // Motor positions
    out << q1d << "," << q2d << "," << q3d << "," << q4d << ","; // Commanded Motor positions
    out << v1 << "," << v2 << "," << v3 << "," << v4 << ",";     // Motor velocities
    out << torque1 << "," << torque2 << "," << torque3 << "," << torque4 << ","; // Motor torques
    out << i << "," << ts.tv_sec << ',';                         // Servo cycle count, timestamp
    out << cur_pos[0] << ','<< cur_pos[1] << ','<< cur_pos[2] << ','<< cur_pos[3] << ','; // Commanded joint positions
    out << cur_pos_act[0] << ','<< cur_pos_act[1] << ','<< cur_pos_act[2] << ','<< cur_pos_act[3]; // Actual joint positions
    log = out.str();
    i++;

    //cout << i << "," << ts.tv_sec<<endl;
    //cout<<log<<endl;

    const char * c= log.c_str();
    char ch[strlen(c)];
    strcpy(ch,c);
    char dest[] = "localhost";
    printu(dest, 10050, ch);
  }
  pthread_exit(NULL);
  //exit(0);
}

static void jog(void){
  int dir = 1;
  if(jogminus)dir = -1;
  /*switch(axis){
  case 1:{
    v1d = dir*3600;
    break;
  }
  case 2:{
    v2d = dir*3600;
    break;
  }
  case 3:{
    v3d = dir*3600;
    break;
  }
  case 4:{
    v4d = dir*3600;
    break;
  }
  default:
    ; // no axis
  }*/
  switch(axis){
  case 1:{
    q1d+=dir*vstep1;
    q2d+=dir*vstep1/2;
    q3d+=dir*vstep1/2;
    break;
  }
  case 2:{
    q3d+=dir*vstep2;
    q2d+=dir*vstep2;
    break;
  }
  case 3:{
    q2d+=dir*vstep3;
    break;
  }
  case 4:{
    q4d+=dir*vstep4;
    break;
  }
  default:
    ; // no axis
    }
}

void delta_p(int axis, double degrees){
    switch(axis){
    case 1:{
            q1d+=degrees;
            q2d+=degrees*0.5;
            q3d+=degrees*0.5;
            break;
        }
    case 2:{
            q3d+=degrees;
            q2d+=degrees;
            break;
        }
    case 3:{
            q2d+=degrees;
            break;
        }
    case 4:{
            q4d+=degrees;
            break;
        }
    default: break;
    }
}

static int kbhit(void){
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

  if(ch != EOF)
    {
      ungetc(ch, stdin);
      return 1;
    }

  return 0;
}

static int printu(char * hostname, int port, char * data){
  int hSock = socket(AF_INET, SOCK_DGRAM, 0);
  if (hSock > -1){
    struct hostent *pServer = gethostbyname(hostname);
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    memcpy(&addr.sin_addr.s_addr, pServer->h_addr, pServer->h_length);
    addr.sin_port = htons(port);

    int n = sendto(hSock, data, strlen(data), 0, (sockaddr*)&addr, sizeof(addr));
    if (n < 0){
      fprintf(stderr, "Error, send() failed: %s\n", strerror(errno));
      shutdown(hSock, SHUT_RDWR);
      close(hSock);
      return 1;
    }
    shutdown(hSock, SHUT_RDWR);
    close(hSock);
    return 0;
  }
  return 0;
}

void * robot_server(void *ptr){   // takes commands from a UDP socket and relays them to the main loop
    int buflen = 512;
    int port = 10051;
    struct sockaddr_in  si_me, si_other;
    int s;
    socklen_t slen = sizeof (si_other);

    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
      diep("socket");

    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(port);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s,(struct sockaddr *) &si_me, sizeof(si_me))==-1)
        diep("bind");

    for(;;){
        memset(cmdbuf,'\0',buflen);
        int size  = recvfrom(s, cmdbuf, buflen, 0, (struct sockaddr *) &si_other, &slen);
        if(size>0)newcmd++;  // When this flag changes, the main loop grabs cmdbuf and runs process_input on the first character
        usleep(5000);
        if(quit)break;
    }
    close(s);
    pthread_exit(NULL);
}

void diep(char *s){
    char dest[] = "localhost";
    printu(dest, 10052, s);

    /*process_input('q');
    pthread_kill(servo, SIGKILL);
    exit_cleanup();
    perror(s);
    exit(1);*/

    exit_cleanup();
    perror(s);
    pthread_kill(servo, SIGTERM);
    exit(1);
}

void diep(const char *s){
    char ch[strlen(s)];
    strcpy(ch,s);
    diep(ch);
}

void err(char *s){
    char dest[] = "localhost";
    printu(dest, 10052, s);

    exit_cleanup();
    printf("Error: %s\r\n",s);
    pthread_kill(servo, SIGTERM);
    exit(1);
}

void err(const char *s){
    char ch[strlen(s)];
    strcpy(ch,s);
    err(ch);
}

void exit_cleanup(void){
    zero_torques();
    S626_InterruptEnable (constants::board0, FALSE);
    S626_CloseBoard(constants::board0);
    cout << "626 closed out." << endl;
}

void serve_request(string command){
    cout<<command<<endl;
    static bool relative = false;  // is the move relative, or absolute
    static bool rapid = true;      // rapid, or interpolated
    bool move = false;             // do we need to move?

    vector<double> dest_pos = get_Joint_Pos();
    vector<double> rel_move(4,0.0);

    char data[512];
    char cmd[10][2];
    double op[10];
    int argc = 0;

    memset(data, 0, 512);
    argc = sscanf(command.c_str(),"%1s%lf%1s%lf%1s%lf%1s%lf%1s%lf%1s%lf%1s%lf%1s%lf%1s%lf%1s%lf",
                  cmd[0], &op[0], cmd[1], &op[1], cmd[2], &op[2], cmd[3], &op[3], cmd[4], &op[4],
                  cmd[5], &op[5], cmd[6], &op[6], cmd[7], &op[7], cmd[8], &op[8], cmd[9], &op[9]);

    for(int i=0;i<argc/2;i++){
        switch(cmd[i][0]){
        case'g':{
                int num = (int)op[i];
                printf("num:%d\r\n",num);
                if(num==90)relative = false;
                if(num==91)relative = true;
                if(num==0)rapid = true;
                if(num==1)rapid = false;
                break;
            }
        case'j':{
                move = true;
                int num = (int)op[i]-1;
                if(!relative)dest_pos[num] = op[i+1];
                if(relative)rel_move[num] = op[i+1];
                //cout<<num<<","<<op[i+1]<<endl;
                i++;
                break;
            }
        default:
            ; // no action
        }
    }
    //printf("%lf,%lf,%lf,%lf\r\n",dest_pos[0],dest_pos[1],dest_pos[2],dest_pos[3]);
    //printf("%lf,%lf,%lf,%lf\r\n",rel_move[0],rel_move[1],rel_move[2],rel_move[3]);

    for(int i = 0;i<4;i++)dest_pos[i]+=rel_move[i];
    set_Joint_Pos(dest_pos);
}

vector<double> get_Joint_Pos(void){
    double motors [] = {q1d,q3d,q2d,q4d};
    double joints [] = {0.0,0.0,0.0,0.0};
    for(int i = 0;i<4;i++){
        for (int j = 0;j<4;j++){
            joints[i]+=constants::m2j[4*i+j]*motors[j];  // Convert motor positions to joint positions
        }
    }
    vector<double> Pos(joints, joints + sizeof(joints));
    return Pos;
}

vector<double> get_Joint_Pos_Actual(void){
    double motors [] = {q1,q3,q2,q4};
    double joints [] = {0.0,0.0,0.0,0.0};
    for(int i = 0;i<4;i++){
        for (int j = 0;j<4;j++){
            joints[i]+=constants::m2j[4*i+j]*motors[j];  // Convert motor positions to joint positions
        }
    }
    vector<double> Pos(joints, joints + sizeof(joints));
    return Pos;
}

void set_Joint_Pos(vector<double> Pos){
    double motors [] = {0.0,0.0,0.0,0.0};
    for(int i = 0;i<4;i++){
        for (int j = 0;j<4;j++){
            motors[i]+=constants::j2m[4*i+j]*Pos[j];  // Convert motor positions to joint positions
        }
    }
    q1d = motors[0];
    q3d = motors[1];
    q2d = motors[2];
    q4d = motors[3];
    //cout<<motors[0]<<','<<motors[1]<<','<<motors[2]<<','<<motors[3]<<endl;
}

vector<double> get_Joint_Vel(void){
    double array[] = { v1, v2 - v3, v2 + v3, v4 };
    vector<double> Vel(array, array + sizeof(array));

    return Vel;

}

//------------------Stuff for testing------------------------------------
void * servo_loop2(void *ptr){
  static uint16_t tlast = 0;
  static int i = 0;
  cout<<"child start"<<endl;

  while(1){
    uint16_t time;
    //uint16_t t;

    pthread_cond_signal(&g_cond);
    pthread_mutex_unlock(&g_mutex);

    do time = S626_CounterReadLatch(constants::board0,constants::cntr_chan);
    while((uint16_t)(time - tlast) < CYCLE_COUNTS);
    tlast = time;
    //usleep(1000);
 if(quit){
       cout<<"done"<<endl;
       break;
    }
    pthread_mutex_lock(&g_mutex);
    //cout<<time<<endl;

    if(i%1000==0)cout<<i<<endl;
    i++;
  }
  cout<<"done1"<<endl;
  //pthread_cond_signal(&g_cond);
  //pthread_mutex_unlock(&g_mutex);
  pthread_exit(NULL);//return NULL;
  cout<<"done2"<<endl;
  exit(0);
}

#ifdef TESTING
int main(int argc, char** argv){
  int result;
  pthread_t       servo;
  pthread_attr_t  attributes;

  pthread_attr_init(&attributes);
  pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_JOINABLE);

  pthread_mutex_init(&g_mutex, NULL);
  pthread_cond_init(&g_cond, NULL);
  pthread_mutex_lock(&g_mutex);

  result = pthread_create(&servo, &attributes, servo_loop, NULL);
  if (result == 0) cout << "Servo thread started." << endl;

  char c = '0';
 while(1){
   pthread_cond_wait(&g_cond, &g_mutex);
      if(kbhit()){
        c = getchar();
        printf("%c\r\n",c);
        rewind(stdout);
        ftruncate(1,0);
        process_input(c);
        if(c == 'q'||c=='Q')break;
        cout << "Kp1: " << Kp1 << "," <<  " Kv1: " << Kv1 << "," << "Kp2: " << Kp2 << "," <<  " Kv2: " << Kv3 << "," <<"Kp4: " << Kp4 << "," <<  " Kv4: " << Kv4 << "," << " cutoff (Hz): " << cutoff<<endl;
      }
    }
  pthread_join(servo, NULL);
  exit(0);
}
#endif
//---------------------------end of stuff for testing--------------------------------------
