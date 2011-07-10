/** Interface for Bosch Arm.
  * 
**/
#ifndef BOSCH_ARM_H
#define BOSCH_ARM_H
#include <stdint.h> //for unit16_t
class BoschArm
{
private:
  double home_offsets[4];
  double ql[4];
  int filter_order;
  double* a;
  double* b;
  double* x_his[4];
  double* y_his[4];
  uint16_t time_now;
  uint16_t time_last; //for wait
  uint16_t time_now2;
  uint16_t time_last2;//for update computing v
  //double r1; // joint 1, turns pers motor turn
  //double r2;  // joint 2-3-4, turns per motor turn
  //double r3;  // joint 1 motor turns per joint turn
  //double r4;  // joint 2-3-4, motor turns per joint turn  
  double L0; //height of joint 1
  double L3; //length of link 3
  double L4; //length of link 4 in x direction
  double L5; //length of link 4 in y direction
  double rad_per_count; //rads per encoder count  
  void safe_exit(const char* msg);
  void enforce_safety();
public:
  double q[4];//actuator position
  double v[4];//actuator speed
  double torque[4];//actuator torque
  double v_lims[4];//speed limit
  double t_lims[4];//torque limit
  double dt;
  double m2j[16];//transmission matrix
  //setup the board and initialize the states.
  void initialize();
  //update q,v and enforce torque.
  void update();
  //reset the states and shut down the board.
  void close(); 
  //wait for a update cycle.
  void wait();
  //convert robot time to common time.
  double convertToWallTime(uint16_t now,uint16_t last);
  //get the hardware time
  uint16_t getTime();
  void motor2JointPosition(const double* motors, double* joints);
  void joint2TipPosition(const double* joints, double* tip);
  void getJacobianJoint2Tip(const double* joints, double* jacobian);
  //a helper function
  void truncate(double &x, double max);  
  
  //   void joint2MotorPosition(const double* joints, double* motors)
//   {
//     for (int i=0;i<4;i++)
//     {
//       motors[i]=0;
//       for (int j=0;j<4;j++)
//         motors[i]+=j2m[4*i+j]*joints[j];
//     }
//   }  
  
};
#endif