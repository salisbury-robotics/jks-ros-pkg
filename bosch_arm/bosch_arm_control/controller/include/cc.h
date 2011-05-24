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

  const double t_max = 0.184;          // N-m - max 100% duty for RE 40
  const double v_for_t_max = 10.0;     // D/A output volts - 10 volts = max torque

  const HBD board = 0;
}
#endif
