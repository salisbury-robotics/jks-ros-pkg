#include "cc.h"

using namespace std;

class Motor{
  double p,pl,pd,v,vl,vd,a,t_lim,acc,Kp,Kv,dt,cnt2deg;
public:
  Motor();

};

Motor::Motor () {
  t_lim = 0.0;
  acc = 0.0;
}

//int main () {
  //CRectangle rect;
  //rect.set_values (3,4);
  //cout << "area: " << rect.area();
  //return 0;
//}
/*
static double t_lim1 = 1.0*constants::t_max;
static double t_lim2 = 1.0*constants::t_max;
static double t_lim3 = 1.0*constants::t_max;
static double t_lim4 = 1.0*constants::t_max;

static bool t_wave1 = false;
static bool t_wave2 = false;
static bool t_wave3 = false;
static bool t_wave4 = false;

static double vstep1 = 0.3;
static double vstep2 = 0.2;
static double vstep3 = 0.05;
static double vstep4 = 1.2;

static double q1, q2, q3, q4 = 0;     // current position, in degrees
static double q1l, q2l, q3l, q4l = 0; // last position
static double q1d, q2d, q3d, q4d = 0; // desired position

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

static int quit = 0;
static int jog_step = 50;
static double lambda = 1.0;
static double cutoff = 1000;
static int axis = 0;
static bool jogplus = false;
static bool jogminus = false;
*/
