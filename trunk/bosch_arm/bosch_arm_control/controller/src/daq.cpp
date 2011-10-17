// 626 native driver stuff
#include "s626drv.h"
#include "App626.h"
#include "s626mod.h"
#include "s626core.h"
#include "s626api.h"

#include "cc.h"
#include "daq.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>

using namespace std;
#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>

static int home_flags[] = {0,0,0,0};

#define BOARD 0
const unsigned int board = 0;
const int v_factor_626 = 819;  //Multiply volts times this number to actually get that voltage on the card.

void InterruptAppISR(DWORD board);
void ErrorFunction1(DWORD ErrFlags);

static int cnt1 = 0; // count for interrupts
static unsigned long errFlags = 0x0; // error flags

pthread_mutex_t	CriticalSection = PTHREAD_MUTEX_INITIALIZER;
DWORD IntCounts[16] = { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0 };	// Interrupt counters.
// end 626 native driver stuff

int setup626(void){
  S626_OpenBoard(board, 0, InterruptAppISR, 1);
  S626_InterruptEnable(board, TRUE);
  S626_SetErrCallback(board, ErrorFunction1);
  errFlags = S626_GetErrors (board);
  printf("Board(%i) -  ErrFlags = 0x%lu \n", board, errFlags);
  CreateEncoderCounters();
  counter_setup();
  return errFlags;
}

void InterruptAppISR(DWORD board){
  WORD intStatus[4];
  register WORD status;
  register DWORD *pCounts;
  register WORD mask;
  BOOL bRE = FALSE;

  S626_InterruptStatus (board, intStatus);		// Fetch IRQ status for all sources.
  if (intStatus[3]&0xfc00) {  // interrupts from all Counter's overflow
    cnt1++;
    //printf("%d",intStatus[3]&0xfc00);
    cout << S626_CounterReadLatch(constants::board0, constants::cntr_chan) << endl;
    S626_CounterCapFlagsReset(constants::board0, constants::cntr_chan);	//reset counter interrupts
    bRE = TRUE;
  }

  if(intStatus[3]&16){ //M1
    home_flags[0] = 1;
    S626_CounterCapFlagsReset(constants::board0, CNTR_0A);
    S626_CounterIntSourceSet (constants::board0, CNTR_0A, INTSRC_NONE);
    S626_CounterEnableSet(constants::board0, CNTR_0A, CLKENAB_ALWAYS);
    S626_CounterModeSet(constants::board0, CNTR_0A, (INDXSRC_SOFT << BF_INDXSRC));
  }
  if(intStatus[3]&64){ //M2
    home_flags[1] = 1;
    S626_CounterCapFlagsReset(constants::board0, CNTR_1A);
    S626_CounterIntSourceSet (constants::board0, CNTR_1A, INTSRC_NONE);
    S626_CounterEnableSet(constants::board0, CNTR_1A, CLKENAB_ALWAYS);
    S626_CounterModeSet(constants::board0, CNTR_1A, (INDXSRC_SOFT << BF_INDXSRC));
  }
  if(intStatus[3]&256){ //M3
    home_flags[2] = 1;
    S626_CounterCapFlagsReset(constants::board0, CNTR_2A);
    S626_CounterIntSourceSet (constants::board0, CNTR_2A, INTSRC_NONE);
    S626_CounterEnableSet(constants::board0, CNTR_2A, CLKENAB_ALWAYS);
    S626_CounterModeSet(constants::board0, CNTR_2A, (INDXSRC_SOFT << BF_INDXSRC));
  }
  if(intStatus[3]&32){ //M4
    home_flags[3] = 1;
    S626_CounterCapFlagsReset(constants::board0, CNTR_0B);
    S626_CounterIntSourceSet (constants::board0, CNTR_0B, INTSRC_NONE);
    S626_CounterEnableSet(constants::board0, CNTR_0B, CLKENAB_ALWAYS);
    S626_CounterModeSet(constants::board0, CNTR_0B, (INDXSRC_SOFT << BF_INDXSRC));
  }

  /*if(intStatus[3]){
    S626_CounterCapFlagsReset(constants::board0, CNTR_0A);
    S626_CounterCapFlagsReset(constants::board0, CNTR_0B);
    S626_CounterCapFlagsReset(constants::board0, CNTR_1A);
    S626_CounterCapFlagsReset(constants::board0, CNTR_2A);
    }*/

printf("Status: %d, %d, %d, %d\r\n",intStatus[0],intStatus[1],intStatus[2],intStatus[3]);

  if(intStatus[0])		// interrupts from DIO channel 0-15
    {
      // Cache a copy of DIO channel 0-15 interrupt request (IRQ) status.
      status = intStatus[0];	// Cache DIO 0-15 IRQ status.
      // Tally DIO 0-15 interrupts.
      pCounts = IntCounts;		// Init pointer to interrupt counter.

      pthread_mutex_lock(&CriticalSection);		// * Start thread-safe section -----------
      for (mask = 1; mask != 0; pCounts++)
	{ // *
	  if (status & mask)	// * If DIO is requesting service ...
	    (*pCounts)++;	// * increment DIOs interrupt counter.
	  mask += mask;			// * Bump mask.
	} // *
      pthread_mutex_unlock(&CriticalSection);	// * End thread-safe section -------------

      // Negate all processed DIO interrupt requests.
      S626_DIOCapReset(constants::board0, 0, status);		// group #0: DIO 0-15

      // Unmask boards master interrupt enable.
      bRE = TRUE;
    }
  if(bRE){
    S626_InterruptEnable(constants::board0, TRUE);			//Re-enable interrupts
  }
}

//***** print error code
void ErrorFunction1 (DWORD ErrFlags){
  printf ("Got an error on board 0 0x%x\n", ErrFlags);
}

void CreateEncoderCounters(void){
   /*
     #define CNTR_0A 			0	//      Counter 0A.
     #define CNTR_1A 			1	//      Counter 1A.
     #define CNTR_2A 			2	//      Counter 2A.
     #define CNTR_0B 			3	//      Counter 0B.
     #define CNTR_1B			4	//      Counter 1B.
     #define CNTR_2B                    5       //      Counter 2B.
   */

  // Reset Counters
   S626_CounterCapFlagsReset(constants::board0, CNTR_0A);
   S626_CounterCapFlagsReset(constants::board0, CNTR_1A);
   S626_CounterCapFlagsReset(constants::board0, CNTR_2A);
   S626_CounterCapFlagsReset(constants::board0, CNTR_0B);
   
   S626_CounterModeSet(constants::board0, CNTR_0A,
			(LOADSRC_INDX << BF_LOADSRC)|    // Preload upon index.
			(INDXSRC_HARD << BF_INDXSRC)|    // Enable hardware index.
			(CLKSRC_COUNTER << BF_CLKSRC)|   // Operating mode is Counter.
			(CLKPOL_POS  << BF_CLKPOL)|      // Active high clock.
			//(CNTDIR_UP << BF_CLKPOL)|      // Count direction is Down.
                        (CLKMULT_4X << BF_CLKMULT)|      // Clock multiplier is 4x.
                        (CLKENAB_ALWAYS << BF_CLKENAB)); // Counting always enabled.

  S626_CounterModeSet(constants::board0, CNTR_1A,
			(LOADSRC_INDX << BF_LOADSRC)|    // Preload upon index.
			(INDXSRC_HARD << BF_INDXSRC)|    // Disable hardware index.
			(CLKSRC_COUNTER << BF_CLKSRC)|   // Operating mode is Counter.
			(CLKPOL_POS << BF_CLKPOL)|       // Active high clock.
			//(CNTDIR_UP << BF_CLKPOL)|      // Count direction is Down.
                        (CLKMULT_4X << BF_CLKMULT)|      // Clock multiplier is 4x.
                        (CLKENAB_ALWAYS << BF_CLKENAB)); // Counting always enabled

  S626_CounterModeSet(constants::board0, CNTR_2A,
			(LOADSRC_INDX << BF_LOADSRC)|    // Preload upon index.
			(INDXSRC_HARD << BF_INDXSRC)|    // Disable hardware index.
			(CLKSRC_COUNTER << BF_CLKSRC)|   // Operating mode is Counter.
			(CLKPOL_POS << BF_CLKPOL)|       // Active high clock.
			//(CNTDIR_UP << BF_CLKPOL)|      // Count direction is Down.
                        (CLKMULT_4X << BF_CLKMULT)|      // Clock multiplier is 4x.
                        (CLKENAB_ALWAYS << BF_CLKENAB)); // Counting always enabled

  S626_CounterModeSet(constants::board0, CNTR_0B,
			(LOADSRC_INDX << BF_LOADSRC)|    // Preload upon index.
			(INDXSRC_HARD << BF_INDXSRC)|    // Disable hardware index.
			(CLKSRC_COUNTER << BF_CLKSRC)|   // Operating mode is Counter.
			(CLKPOL_POS << BF_CLKPOL)|       // Active high clock.
			//(CNTDIR_UP << BF_CLKPOL)|      // Count direction is Down.
                        (CLKMULT_4X << BF_CLKMULT)|      // Clock multiplier is 4x.
                        (CLKENAB_ALWAYS << BF_CLKENAB)); // Counting always enabled
	
   // Set counter core and preload value to be mid of 2^24 (since all counters are 24-bit), so as to make test easy.
   S626_CounterPreload(constants::board0, CNTR_0A, 8388608);	// 0x800000 = 2^24/2 = 8388608
   S626_CounterPreload(constants::board0, CNTR_1A, 8388608);	// 0x800000 = 2^24/2 = 8388608
   S626_CounterPreload(constants::board0, CNTR_2A, 8388608);	// 0x800000 = 2^24/2 = 8388608
   S626_CounterPreload(constants::board0, CNTR_0B, 8388608);	// 0x800000 = 2^24/2 = 8388608

   //S626_CounterSoftIndex(constants::board0, CNTR_0A);		// Generate a index signal by software,
   //S626_CounterSoftIndex(constants::board0, CNTR_1A);		// Generate a index signal by software,
   //S626_CounterSoftIndex(constants::board0, CNTR_2A);		// Generate a index signal by software,
   //S626_CounterSoftIndex(constants::board0, CNTR_0B);		// Generate a index signal by software,

   // Enable latching of accumulated counts on demand.
   S626_CounterLatchSourceSet(constants::board0, CNTR_0A, LATCHSRC_AB_READ);
   S626_CounterLatchSourceSet(constants::board0, CNTR_1A, LATCHSRC_AB_READ);
   S626_CounterLatchSourceSet(constants::board0, CNTR_2A, LATCHSRC_AB_READ);
   S626_CounterLatchSourceSet(constants::board0, CNTR_0B, LATCHSRC_AB_READ);

   S626_CounterIntSourceSet (constants::board0, CNTR_0A, INTSRC_INDX);
   S626_CounterIntSourceSet (constants::board0, CNTR_1A, INTSRC_INDX);
   S626_CounterIntSourceSet (constants::board0, CNTR_2A, INTSRC_INDX);
   S626_CounterIntSourceSet (constants::board0, CNTR_0B, INTSRC_INDX);

   // Enable the counter.
   /*S626_CounterEnableSet(constants::board0, CNTR_0A, CLKENAB_ALWAYS);
   S626_CounterEnableSet(constants::board0, CNTR_1A, CLKENAB_ALWAYS);
   S626_CounterEnableSet(constants::board0, CNTR_2A, CLKENAB_ALWAYS);
   S626_CounterEnableSet(constants::board0, CNTR_0B, CLKENAB_ALWAYS);*/
 }


void counter_setup(void){
  // Reset Counter 2A
  S626_CounterCapFlagsReset (constants::board0, constants::cntr_chan);
  // Set counter operating mode

  S626_CounterModeSet (constants::board0, constants::cntr_chan,
		       (LOADSRC_INDX << BF_LOADSRC) |	// Index causes preload
		       (INDXSRC_SOFT << BF_INDXSRC) |	// Hardware index disabled
		       (CLKSRC_TIMER << BF_CLKSRC) |	// Operating mode is Timer
		       //(CNTDIR_DOWN << BF_CLKPOL) |		// Counting direction is Down
		       (CLKMULT_1X << BF_CLKMULT) |		// Clock multiplier is 1x
		       (CLKENAB_INDEX << BF_CLKENAB));	// Counting is initially disabled

  S626_CounterEnableSet (constants::board0, constants::cntr_chan, CLKENAB_ALWAYS);
}

int read_encoder(int i){
    return S626_CounterReadLatch(constants::board0,i)-8388608;
}

void write_torque(int channel, float torque){
  float v_torque_cmd = (torque/constants::t_max) * constants::v_for_t_max;  //Convert to appropriate output voltage
  //cout<<(int)((float)constants::v_factor*v_torque_cmd)<<","<<v_torque_cmd<<","<<torque<<endl;
  S626_WriteDAC (constants::board0, channel, (int)((float)constants::v_factor*v_torque_cmd));
}

void zero_torques(void){
  write_torque(0, 0);
  write_torque(1, 0);
  write_torque(2, 0);
  write_torque(3, 0);
}

int homed(int motor){
  return home_flags[motor];
}


