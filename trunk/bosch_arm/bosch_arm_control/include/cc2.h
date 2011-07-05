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
  const double r1 = 0.062811565304088; // joint 1, turns pers motor turn
  const double r2 = 0.12524850894632;  // joint 2-3-4, turns per motor turn
  const double r3 = 15.9206349206349;  // joint 1 motor turns per joint turn
  const double r4 = 7.98412698412698;  // joint 2-3-4, motor turns per joint turn
const double m2j[16] =              // Motor to joint position conversion matrix
{
  r1, 0.0, 0.0, 0.0,
  -r1, 0.0,  r2, 0.0,
  0.0, r2,  -r2, 0.0,
  0.0, 0.0, 0.0,  r2
};
const double j2m [16] =              // Joint to motor position conversion matrix
{
  r3, 0.0, 0.0, 0.0,
  r4,  r4,  r4, 0.0,
  r4,  r4, 0.0, 0.0,
  0.0, 0.0, 0.0,  r4
};

  const double t_max = 0.184;          // N-m - max @ 100% duty for RE 40
  const double v_for_t_max = 10.0;     // D/A output volts - 10 volts = max torque
  const double v_lim = 2160;           // motor speed limit, 2160 degrees per second
  const double p_err_lim = 360;        // position err limited to 1 motor revolution
  
  const double L0=0.27;
  const double L3=0.50;
  const double L4=0.48;
  const double L5=0.05;
  const double rad_per_count=1.25663706e-3;
  const HBD board = 0;
  const double pi=3.14159265358979;
}
#endif
