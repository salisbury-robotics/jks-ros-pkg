// constants and conversions
#ifndef C_AND_C
#define C_AND_C

#include "s626drv.h"
#include "App626.h"
#include "s626mod.h"
#include "s626core.h"
#include "s626api.h"

namespace constants{

  const DWORD board0 = 0;
  const float v_factor = 819;       // Multiply volts times this number to actually get that voltage on the card.
  const int timer_hz = 2000000;     // DAQ card clock rate
  const int cycle_hz = 1000;        // Servo cycle rate
  const int cycle_counts = timer_hz/cycle_hz;  // this many counts per servo cycle
  const double cnt2sec = 1/(double)timer_hz;
  const double loop_time = 1/cycle_hz;

  const unsigned long errFlags  = 0x0; // error flags
  const WORD cntr_chan = CNTR_2B;      // servo loop counter
  const double cnt2mdeg = 0.072;       // 360 motor degrees every 5000 counts
  const double r1 = 15.9206349206349;  // motor 1, motor turns per joint turn
  const double r2 = 7.98412698412698;  // motor 2, motor turns per joint turn
  const double r3 = 15.9206349206349;  // motor 3, motor turns per joint turn
  const double r4 = 7.98412698412698;  // motor 4, motor turns per joint turn
//   const double r1 = 15.88;
//   const double r2 = 7.96;
//   const double r3 = 15.88;
//   const double r4 = 7.64;
  const double c24=-0.15;               // coupling between joints 2 and 4

//   const double m2j[16] ={              // Motor to joint position conversion matrix
//        1/r1,  0.0,    0.0,  0.0,
//       -1/r1,  0.0,   1/r3,  0.0,
//         0.0, 1/r2,  -1/r3,  0.0,
//     -c24/r1,  0.0, c24/r3, 1/r4
//   };
//   const double m2j[16] ={              // Motor to joint position conversion matrix
//        1/r1,  0.0,    0.0,  0.0,
//       -1/r1,  0.0,   -1/r3,  0.0,
//         0.0, 1/r2,  1/r3,  0.0,
//     -c24/r1,  0.0, -c24/r3, 1/r4
//   };
//   const double j2m [16] ={              // Joint to motor position conversion matrix
//        r1,     0.0, 0.0, 0.0,
//        r2,      r2,  r2, 0.0,
//        -r3,      -r3, 0.0, 0.0,
//       0.0, -c24*r4, 0.0,  r4
//   };

  const double m2j[]={
    0.0627,         0,         0,         0,
   -0.0617,         0,   -0.0626,         0,
   -0.0005,    0.1255,    0.0626,         0,
    0.0090,         0,    0.0092,    0.1295
  };
 
  const double j2m [16] ={              // Joint to motor position conversion matrix
       15.94,   0.0,      0.0,    0.0,
        7.90,   7.97,     7.97,  0.0,
      -15.71,   -15.98,   0.0,    0.0,
         0.0,   1.13,     0.0,    7.72 
  };
  
  const double t_max = 0.184;          // N-m - max @ 100% duty for RE 40
  const double v_for_t_max = 10.0;     // D/A output volts - 10 volts = max torque
  const double v_lim = 80;           // motor speed limit, 2160 degrees per second
  const double p_err_lim = 360;        // position err limited to 1 motor revolution
  
  const double L0=0.27;
  const double L3=0.50;
  const double L4=0.48;
  const double L5=0.05;
  const double rad_per_count=1.25663706e-3;
  const HBD board = 0;
  const double pi=3.14159265358979;
  //the offset from the zero position in forward kinematics to the fixer.
  //const double q_off[4]={-1.611620+0.130237, 0.0643001+0.0602076, -1.0000682+0.0171557, -2.3054768+0.0103879};
  //const double q_off[4]={-1.611620, 0.0643001, -1.0000682, -2.3054768};
  //   const double gravc[8]={-0.0375042,0.0186818,
  // -0.0305367,-0.620683,
  // -0.0792363,-0.00757456,
  // 0.161804,0.0102816,
  // };
  
  //para for long arm
//  const double q_off[4]={-1.611620+0.130237, 0.0643001+0.0602076, -1.0000682+0.0171557, -2.3054768+0.0103879};
//   const double gravc[8]={0.00,0.00,
// 			 -0.02,-0.33,
// 			 -0.06,0.00,
// 			 0.17,0.00};
//   const double link[8]=
//     {-0.0359,
//      0.0023,
//      -0.0272,
//      -0.5078,
//      -0.0381,
//      0.0142,
//      0.4815,
//      0.0132
//     };



// para for the scanner arm
//const double q_off[4]={0,0,0,0};


//const double q_off[4]={2.15108,-0.290944,-2.60937,3.10153};

//     const double gravc[8]={0.00,0.00,
//        -0.0,-1.07,
//        -0.13,0.00,
//        0.40,0.01};
const double q_off[4]={1.57079633,0,0,1.57079633};

    const double gravc[8]={0.00,0.00,
       -0.0,-1.07,
       -0.13,0.00,
       0.40,0.02};

    //Z0 is up, X0 is forward, Y0 is to the left. O0 is at the bottom of the base.   
    //For i=1,2,3,4   
    //Zi is consistent with the direction of rotation  at joint i  
    //Xi=Zi cross Z(i+1)
    //Yi=Zi cross Xi=-Z(i+1)
    //Oi is at the intersection of Zi and Z(i+1)
    
    //a frame is a point that has position and orientation.
    //frame index convention for frm_home, frm0, frm_cur: -1 world, 0 base, 1-4 R-joints, 5 tip.  
    
    
    const double home_frame_p[]=
    {
     0,     0,    0, 
     0,     0,    0.27,
     0,     0,    0,
     -0.02, -0.50,0,   
     -0.045,0,    0,
     0.185, 0.01, 0     
    };
    
    //the scanner position relative to the tip when the arm is in zero position.
    const double scan_p[]={0,     0,    0};
    
    const double home_frame_M[]=
    {
      1,  0,  0,
      0,  1,  0,
      0,  0,  1,
      
      0,  1,  0,
      0,  0,  1,
      1,  0,  0,
                         
      0,  -1, 0,
      0,  0,  -1,
      1,  0,  0,
      
      0,  -1, 0,
      0,  0,  -1,
      1,  0,  0,
      
      0,  -1, 0,
      0,  0,  -1,
      1,  0,  0,
      
      1,  0,  0,
      0,  1,  0,
      0,  0,  1,
    };
    
     
}
#endif
