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

#include <ros/ros.h>
#include "bosch_arm_srvs/SetJointAngles.h"
#include "sensor_msgs/JointState.h"
#include <boost/thread/thread.hpp>

using namespace std;

ros::Publisher joint_state_pub;
pthread_mutex_t g_mutex;
pthread_cond_t  g_cond;

static int ord[4]={0,1,3,2};

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



void * servo_loop(void *ptr);

void delta_p(int axis, double degrees);
void diep(char *s);
void diep(const char *s);
void err(char *s);
void err(const char *s);
void * robot_server(void *ptr);
void exit_cleanup(void);
//void serve_request(string command);
//vector<double> get_Joint_Pos_Actual(void);
//static int kbhit(void);
static int printu(char * hostname, int port, char * data);


//static double t_lim = {0.4*constants::t_max,0.4*constants::t_max,
//0.4*constants::t_max,0.4*constants::t_max};
// static double t_lim2 = ;
// static double t_lim3 = 0.4*constants::t_max;
// static double t_lim4 = 0.4*constants::t_max;

// static bool t_wave1 = false;
// static bool t_wave2 = false;
// static bool t_wave3 = false;
// static bool t_wave4 = false;

// static double vstep1 = 0.15;
// static double vstep2 = 0.1;
// static double vstep3 = 0.025;
// static double vstep4 = 0.06;//1.2;

//the intial encoder reading
static double home_offsets [4] = {0.0,0.0,0.0,0.0};

//old
static double qhome[]={0,0,0,0};
static double q[]={0,0,0,0};
static double ql[]={0,0,0,0};
static double qd[]={0,0,0,0};
// static double q2home = 0;
// static double q3home = 0;
// static double q4home = 0;

// static double q1, q1l, q1d = q1home;
// static double q2, q2l, q2d = q2home;
// static double q3, q3l, q3d = q3home;
// static double q4, q4l, q4d = q4home;

//static double v1, v2, v3, v4 = 0;     // deg/s
static double v[]={0,0,0,0};
//static double v1l, v2l, v3l, v4l = 0; // last velocity
static double vl[]={0,0,0,0};
//static double v1d, v2d, v3d = 0; // desired velocity
//static double v4d = 360; // desired velocity

// static double acc1 = 300;             // deg/s^2
// static double acc2 = 300;
// static double acc3 = 300;
// static double acc4 = 300;
static double torque[]={0,0,0,0};;
static timespec ts; //current system time

static double Kp[] = {0.013,0.013,0.013,0.013}; //0.025;
static double Kv[] = {0.0001,0.0001,0.0001,0.0001}; //0.0000091;
// static double Kp2 = 0.013;
// static double Kv2 = 0.0001; //0.0000091;
// static double Kp3 = 0.013;
// static double Kv3 = 0.0001; //0.0000091;
// static double Kp4 = 0.013; //0.036;
// static double Kv4 = 0.0001; //0.000011;

// static bool joint_space = true;
// static bool flail_around = false;
// static double flail_start = 0.0;
//static double lastpos1, lastpos2, lastpos3, lastpos4 = 0.0;


static int quit = 0;
//static int jog_step = 50;
static double lambda = 1.0;
static double cutoff = 1000;
//static int axis = 0;
//static bool jogplus = false;
//static bool jogminus = false;

static double loop_time = 0.001;
static int newcmd = 0;
static char cmdbuf[512];

static vector<double> cur_pos;
static vector<double> cur_pos_act;

enum Mode { NONE, JOG, INCREMENT, KP, KV, FILTER };
//static Mode mode = NONE;

pthread_t servo;
pthread_t robot;

void pubJointStates() {


    sensor_msgs::JointState js;
    js.name.resize(4);
    js.name[0]="joint1";
    js.name[1]="joint2";
    js.name[2]="joint3";
    js.name[3]="joint4";
    int seq=0;

    while (ros::ok()) {
		boost::this_thread::interruption_point();
		js.position=vector<double>(q,q+4);
		js.velocity=vector<double>(v,v+4);
		js.effort=vector<double>(torque,torque+4);      
        js.header.stamp.sec=ts.tv_sec;
        js.header.stamp.nsec=ts.tv_nsec;
        js.header.seq=seq++;
        js.header.frame_id="";
        joint_state_pub.publish(js);
		usleep(5000);
    }
    return;
}


bool set_joint_angles_srv(bosch_arm_srvs::SetJointAngles::Request &req,
                          bosch_arm_srvs::SetJointAngles::Response &res)
{
//     int hSock = socket(AF_INET, SOCK_DGRAM, 0);
//     if (hSock <=-1)
//         return 1;
//     struct hostent *pServer = gethostbyname("localhost");
//     struct sockaddr_in addr;
//     memset(&addr, 0, sizeof(addr));
// 
//     addr.sin_family = AF_INET;
//     memcpy(&addr.sin_addr.s_addr, pServer->h_addr, pServer->h_length);
//     addr.sin_port = htons(10051);
//     char buf[100];
//     sprintf(buf,"%%g90j1p%fj2p%fj3p%fj4p%f",req.joint_angles[0],req.joint_angles[1],
//             req.joint_angles[2],req.joint_angles[3]);
//     int n = sendto(hSock, buf, strlen(buf), 0, (sockaddr*)&addr, sizeof(addr));
//     if (n < 0) {
//         ROS_ERROR("Sending command %s failed.",buf);
//         shutdown(hSock, SHUT_RDWR);
//         close(hSock);
//         return false;
//     }
	//compute desired motor position
	
	//set_Joint_Pos(req.joint_angles);
	
    return true;
}


//TODO: subscribe to command of type JointTrajectory
//see http://www.ros.org/wiki/robot_mechanism_controllers/JointSplineTrajectoryController
int main(int argc, char** argv){
    static int cmd = newcmd;
    static bool streaming = false;
	ros::init(argc, argv, "bosch_arm_node");
    ros::NodeHandle n;
    
    if(!setup626()){
        // home, must do before starting servo loop
        cout<<"Position the arm at the zero position, then hit Enter to continue"<<endl;
        char ch;
        cin.get(ch);
		
		for(int i=0;i<4;i++)
			home_offsets[i]=read_encoder(ord[i])*constants::cnt2mdeg;

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

        zero_torques();

		joint_state_pub =  n.advertise<sensor_msgs::JointState>("/joint_states",100);
		ros::ServiceServer service = n.advertiseService("set_joint_angles", set_joint_angles_srv);
		boost::thread t2 = boost::thread::thread(boost::bind(&pubJointStates));
		
        // run
        while(ros::ok()){
			
            pthread_cond_wait(&g_cond, &g_mutex);
			ros::spinOnce();
            
        }
        //no data is kept after termination, no need to join children threads.
        exit_cleanup();
    }
}

double clipping(double x,double m){
	if(x>m)
		x=m;
	if(x<-m)
		x=-m;
	return x;
}	

//servo_loop reads qd, encoder, and writes q v ql vl torque
void * servo_loop(void *ptr){
  static uint16_t tlast = 0;
  static timespec tl;//,ts;
  clock_gettime(CLOCK_REALTIME, &tl); 
  while(1){
    uint16_t time; //current chip time
    uint16_t t;	//should be equal to period in cycles.

    pthread_cond_signal(&g_cond);
    pthread_mutex_unlock(&g_mutex);
	//wait for the new cycle
    do time = S626_CounterReadLatch(constants::board0,constants::cntr_chan);
    while((uint16_t)(time - tlast) < CYCLE_COUNTS);

    if(quit)break;  // Don't move this outside the g_cond/g_mutex manipulations
    pthread_mutex_lock(&g_mutex);
    
    t = (time - tlast);
    float dt = (float)t*CNT2SEC; //period in seconds
    tlast = time;
    clock_gettime(CLOCK_REALTIME, &ts);
    // read in positions, in motor degrees
    q[0] = -read_encoder(0)*constants::cnt2mdeg + home_offsets[0];
    q[1] = -read_encoder(1)*constants::cnt2mdeg + home_offsets[1];
    q[2] = read_encoder(3)*constants::cnt2mdeg - home_offsets[2]; // <-- notice, out of order, need to fix wiring at some point
    q[3] = read_encoder(2)*constants::cnt2mdeg - home_offsets[3];

    // bail if something is too wrong
    bool bail = false;

    // position error limit
	float dq[4];
	float maxq[4];
	double e_lim = constants::p_err_lim;
	double t_lim1 = 0.4*constants::t_max;
	for(int i=0;i<4;i++){
		dq[i]=qd[i]-q[i];
		if(dq[i]>e_lim||-dq[i]>e_lim){
			bail=true;
			err("position error too large");
		}
		maxq[i]=1.0*t_lim1/Kp[i];
		dq[i]=clipping(dq[i],maxq[i]);
	}


    // set up velocity filter
    lambda = exp(-2*3.14*cutoff*loop_time);

	
    // calculate velocities
	for(int i=0;i<4;i++)
		v[i]=(q[i]-ql[i])/dt*(1-lambda)+lambda*v[i];


    // velocity limit
    double vlim = constants::v_lim;

    if(v[1] > vlim || v[2] > 2*vlim || v[3] > 2*vlim || v[4] > 2*vlim)bail = true;
    if(-v[1] > vlim || -v[2] > 2*vlim || -v[3] > 2*vlim || -v[4] > 2*vlim)bail = true;
    if(bail)err("overspeed");

    // save last velocities
	double t_lim = constants::t_max;
	for(int i=0;i<4;i++){
		ql[i]=q[i];
		vl[i]=v[i];
		torque[i]=Kp[i]*dq[i]-Kv[i]*v[i];
		if(torque[i]>5*t_lim||torque[i]<-5*t_lim){
			bail=true;
			err("torque command over 5 times max achievable");
		}
		torque[i]=clipping(torque[i],t_lim1);
	}



    // tell DAQ card to output torques
	for(int i=0;i<4;i++)
    	write_torque(i,torque[i]);

    // datalogging
    std::string log;
    std::stringstream out;
	for(int i=0;i<4;i++)
		out<<q[i]<<",";
	for(int i=0;i<4;i++)
		out<<qd[i]<<",";
	for(int i=0;i<4;i++)
		out<<v[i]<<",";
	for(int i=0;i<4;i++)
		out<<torque[i]<<",";
    out << ts.tv_sec << ','<< ts.tv_nsec;                         // Servo cycle count, timestamp
    log = out.str();

    const char * c= log.c_str();
    char ch[strlen(c)];
    strcpy(ch,c);
    char dest[] = "localhost";
    printu(dest, 10050, ch);
  }
  pthread_exit(NULL);
  //exit(0);
}



// static void jog(void){
//   int dir = 1;
//   if(jogminus)dir = -1;
//  
//   switch(axis){
//   case 1:{
//     q1d+=dir*vstep1;
//     q2d+=dir*vstep1/2;
//     q3d+=dir*vstep1/2;
//     break;
//   }
//   case 2:{
//     q3d+=dir*vstep2;
//     q2d+=dir*vstep2;
//     break;
//   }
//   case 3:{
//     q2d+=dir*vstep3;
//     break;
//   }
//   case 4:{
//     q4d+=dir*vstep4;
//     break;
//   }
//   default:
//     ; // no axis
//     }
// }

// void delta_p(int axis, double degrees){
//     switch(axis){
//     case 1:{
//             q1d+=degrees;
//             q2d+=degrees*0.5;
//             q3d+=degrees*0.5;
//             break;
//         }
//     case 2:{
//             q3d+=degrees;
//             q2d+=degrees;
//             break;
//         }
//     case 3:{
//             q2d+=degrees;
//             break;
//         }
//     case 4:{
//             q4d+=degrees;
//             break;
//         }
//     default: break;
//     }
// }

// static int kbhit(void){
//   struct termios oldt, newt;
//   int ch;
//   int oldf;
// 
//   tcgetattr(STDIN_FILENO, &oldt);
//   newt = oldt;
//   newt.c_lflag &= ~(ICANON | ECHO);
//   tcsetattr(STDIN_FILENO, TCSANOW, &newt);
//   oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
//   fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
// 
//   ch = getchar();
// 
//   tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
//   fcntl(STDIN_FILENO, F_SETFL, oldf);
// 
//   if(ch != EOF)
//     {
//       ungetc(ch, stdin);
//       return 1;
//     }
// 
//   return 0;
// }

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

// void * robot_server(void *ptr){   // takes commands from a UDP socket and relays them to the main loop
//     int buflen = 512;
//     int port = 10051;
//     struct sockaddr_in  si_me, si_other;
//     int s;
//     socklen_t slen = sizeof (si_other);
// 
//     if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
//       diep("socket");
// 
//     memset((char *) &si_me, 0, sizeof(si_me));
//     si_me.sin_family = AF_INET;
//     si_me.sin_port = htons(port);
//     si_me.sin_addr.s_addr = htonl(INADDR_ANY);
//     if (bind(s,(struct sockaddr *) &si_me, sizeof(si_me))==-1)
//         diep("bind");
// 
//     for(;;){
//         memset(cmdbuf,'\0',buflen);
//         int size  = recvfrom(s, cmdbuf, buflen, 0, (struct sockaddr *) &si_other, &slen);
//         if(size>0)newcmd++;  // When this flag changes, the main loop grabs cmdbuf and runs process_input on the first character
//         usleep(5000);
//         if(quit)break;
//     }
//     close(s);
//     pthread_exit(NULL);
// }

void diep(char *s){
    char dest[] = "localhost";
    printu(dest, 10052, s);

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

// void serve_request(string command){
// 	
//     //cout<<command<<endl;
//     static bool relative = false;  // is the move relative, or absolute
//     static bool rapid = true;      // rapid, or interpolated
//     bool move = false;             // do we need to move?
// 
//     vector<double> dest_pos = get_Joint_Pos();
//     vector<double> rel_move(4,0.0);
// 
//     char data[512];
//     char cmd[10][2];
//     double op[10];
//     int argc = 0;
// 
//     memset(data, 0, 512);
// 
//     argc = sscanf(command.c_str(),"%1s%lf%1s%lf%1s%lf%1s%lf%1s%lf%1s%lf%1s%lf%1s%lf%1s%lf%1s%lf",
//                   cmd[0], &op[0], cmd[1], &op[1], cmd[2], &op[2], cmd[3], &op[3], cmd[4], &op[4],
//                   cmd[5], &op[5], cmd[6], &op[6], cmd[7], &op[7], cmd[8], &op[8], cmd[9], &op[9]);
// 
//     for(int i=0;i<argc/2;i++){
//         switch(cmd[i][0]){
//         case'g':{
//                 int num = (int)op[i];
//                 //printf("num:%d\r\n",num);
//                 if(num==90)relative = false;
//                 if(num==91)relative = true;
//                 if(num==0)rapid = true;
//                 if(num==1)rapid = false;
//                 break;
//             }
//         case'j':{
//                 move = true;
//                 int num = (int)op[i]-1;
//                 if(!relative)dest_pos[num] = op[i+1];
//                 if(relative)rel_move[num] = op[i+1];
//                 //cout<<num<<","<<op[i+1]<<endl;
//                 i++;
//                 break;
//             }
//         default:
//             ; // no action
//         }
//     }
// //     printf("%lf,%lf,%lf,%lf\r\n",dest_pos[0],dest_pos[1],dest_pos[2],dest_pos[3]);
// //     printf("%lf,%lf,%lf,%lf\r\n",rel_move[0],rel_move[1],rel_move[2],rel_move[3]);
// 
//     for(int i = 0;i<4;i++)dest_pos[i]+=rel_move[i];
//     set_Joint_Pos(dest_pos);
// 
// }

// vector<double> get_Joint_Pos(void){
//     double motors [] = {q1d,q3d,q2d,q4d};
//     double joints [] = {0.0,0.0,0.0,0.0};
//     for(int i = 0;i<4;i++){
//         for (int j = 0;j<4;j++){
//             joints[i]+=constants::m2j[4*i+j]*motors[j];  // Convert motor positions to joint positions
//         }
//     }
//     vector<double> Pos(joints, joints + sizeof(joints));
//     return Pos;
// }
// 
// vector<double> get_Joint_Pos_Actual(void){
//     double motors [] = {q1,q3,q2,q4};
//     double joints [] = {0.0,0.0,0.0,0.0};
//     for(int i = 0;i<4;i++){
//         for (int j = 0;j<4;j++){
//             joints[i]+=constants::m2j[4*i+j]*motors[j];  // Convert motor positions to joint positions
//         }
//     }
//     vector<double> Pos(joints, joints + sizeof(joints));
//     return Pos;
// }
// 
// void set_Joint_Pos(vector<double> Pos){
//     double motors [] = {0.0,0.0,0.0,0.0};
//     for(int i = 0;i<4;i++){
//         for (int j = 0;j<4;j++){
//             motors[i]+=constants::j2m[4*i+j]*Pos[j];  // Convert motor positions to joint positions
//         }
//     }
//     q1d = motors[0];
//     q3d = motors[1];
//     q2d = motors[2];
//     q4d = motors[3];
//     //cout<<motors[0]<<','<<motors[1]<<','<<motors[2]<<','<<motors[3]<<endl;
// }
// 
// vector<double> get_Joint_Vel(void){
//     double array[] = { v1, v2 - v3, v2 + v3, v4 };
//     vector<double> Vel(array, array + sizeof(array));
// 
//     return Vel;
// 
// }

// vector<double> getMotorPos(vector<double>& j){
// 	vector<double> m(4);
// 	for(int i = 0;i<4;i++){
//         for (int j = 0;j<4;j++){
//             m[i]+=constants::j2m[4*i+j]*Pos[j];  // Convert motor positions to joint positions
//         }
//     }
//     
// }
