//////////////////////////////////////////////////////////////////////////////////////////////
// Module    : s626core.c																	//
// Version   : 2.32																			//
// Function  : Core objects for Sensoray Model 626.											//
// Author    : Jim Lamberson and Charlie Liu												//
// Revision  :																				//
//             Mar., 2003   Jim Lamberson	initial (Ver 2.31)								//
//             Feb., 2004   Charlie X. Liu	added more S626CORE_???vvv() functions.			//
//             Feb., 2004   Charlie X. Liu	added counter object constructor fixes.			//
//             Mar., 2004   Charlie X. Liu	moved all CriticalBegin & CriticalEnd to CORE	//
//											level to avoid recursive CriticalSection entry.	//
//             Mar., 2004   Charlie X. Liu	added fixes for CountersCreate() and			//
//											S626CORE_CoreOpen().							//
//             Apr., 2004   Charlie X. Liu	fixed the bug in ReadLatch().					//
//             Feb, 2007    Dean Anderson       64bit Linux supported
// Copyright : (C) 2003 - 2007 Sensoray							
//
//////////////////////////////////////////////////////////////////////////////////////////////

#include <assert.h>
#include "s626mod.h"							// Module-level function prototypes.
#include "s626core.h"							// Core-level function prototypes for consistency check.


// PCI bus interface types.
#define INTEL					1				// Intel bus type.
#define MOTOROLA				2				// Motorola bus type.

/////////////////////////////////////////////////////////////////////////////////
#define PLATFORM				INTEL			// *** SELECT PLATFORM TYPE ***
/////////////////////////////////////////////////////////////////////////////////

#define TRUE					1
#define FALSE					0

#define NULL			(void*) 0

#define EOPL					0x80			// End of ADC poll list marker.

// Error codes that must be visible to this base class.
#define ERR_ILLEGAL_PARM		0x00010000		// Illegal function parameter value was specified.
#define ERR_I2C					0x00020000		// I2C error.
#define ERR_COUNTERSETUP		0x00200000		// Illegal setup specified for counter channel.
#define ERR_DEBI_TIMEOUT		0x00400000		// DEBI transfer timed out.

// Physical DMA buffer size in bytes.
#define DMABUF_SIZE				4096			// DMA buffer size.

// Organization (physical order) and size (in DWORDs) of logical DMA buffers contained by ANA_DMABUF.
#define ADC_DMABUF_DWORDS		40				// ADC DMA buffer must hold 16 samples, plus pre/post garbage samples.
#define DAC_WDMABUF_DWORDS		1				// DAC output DMA buffer holds a single sample.
												// All remaining space in 4KB DMA buffer is available for the RPS1 program.

// Address offsets, in DWORDS, from base of DMA buffer.
#define DAC_WDMABUF_OS			ADC_DMABUF_DWORDS

// Interrupt enab bit in ISR and IER.
#define IRQ_MASK				0x00000040		// IRQ enable for GPIO3.
#define ISR_AFOU				0x00000800		// Audio fifo under/overflow detected.

// RPS command codes.
#define RPS_CLRSIGNAL			0x00000000		// CLEAR SIGNAL
#define RPS_SETSIGNAL			0x10000000		// SET SIGNAL
#define RPS_NOP					0x00000000		// NOP
#define RPS_PAUSE				0x20000000		// PAUSE
#define RPS_UPLOAD				0x40000000		// UPLOAD
#define RPS_JUMP				0x80000000		// JUMP
#define RPS_LDREG				0x90000100		// LDREG (1 U32 only)
#define RPS_STREG				0xA0000100		// STREG (1 U32 only)
#define RPS_STOP				0x50000000		// STOP

#define RPS_LOGICAL_OR			0x08000000		// Logical OR conditionals.
#define RPS_INVERT				0x04000000		// Test for negated semaphores.

#define RPS_DEBI				0x00000002		// DEBI done
#define RPS_SIG0				0x00200000		// RPS semaphore 0 (used by ADC).
#define RPS_SIG1				0x00400000		// RPS semaphore 1 (used by DAC).
#define RPS_SIG2				0x00800000		// RPS semaphore 2 (not used).
#define RPS_GPIO2				0x00080000		// RPS GPIO2

#define RPS_SIGADC				RPS_SIG0		// Trigger/status for ADC's RPS program.
#define RPS_SIGDAC				RPS_SIG1		// Trigger/status for DAC's RPS program.

// RPS clock parameters.
#define RPSCLK_SCALAR			8						// This is apparent ratio of PCI/RPS clks (undocumented!!).
#define RPSCLK_PER_US			( 33 / RPSCLK_SCALAR )	// Number of RPS clocks in one microsecond.

// Event counter source addresses.
#define SBA_RPS_A0				0x27			// Time of RPS0 busy, in PCI clocks.

// GPIO constants.
#define GPIO_BASE				0x10004000		// GPIO 0,2,3 = inputs, GPIO3 = IRQ; GPIO1 = out.
#define GPIO1_LO				0x00000000		// GPIO1 set to LOW.
#define GPIO1_HI				0x00001000		// GPIO1 set to HIGH.

// Primary Status Register (PSR) constants.
#define PSR_DEBI_E				0x00040000      // DEBI event flag.
#define PSR_DEBI_S				0x00080000      // DEBI status flag.
#define PSR_A2_IN				0x00008000		// Audio output DMA2 protection address reached.
#define PSR_AFOU				0x00000800		// Audio FIFO under/overflow detected.
#define PSR_GPIO2				0x00000020		// GPIO2 input pin: 0=AdcBusy, 1=AdcIdle.
#define PSR_EC0S				0x00000001		// Event counter 0 threshold reached.

// Secondary Status Register (SSR) constants.
#define SSR_AF2_OUT				0x00000200		// Audio 2 output FIFO under/overflow detected.

// Master Control Register 1 (MC1) constants.
#define MC1_SOFT_RESET			        0x80000000		// Invoke 7146 soft reset.
#define MC1_SHUTDOWN			        0x3FFF0000		// Shut down all MC1-controlled enables.

#define MC1_ERPS1				0x2000			// enab/disable RPS task 1.
#define MC1_ERPS0				0x1000			// enab/disable RPS task 0.
#define MC1_DEBI				0x0800			// enab/disable DEBI pins.
#define MC1_AUDIO				0x0200			// enab/disable audio port pins.
#define MC1_I2C					0x0100			// enab/disable I2C interface.
#define MC1_A2OUT				0x0008			// enab/disable transfer on A2 out.
#define MC1_A2IN				0x0004			// enab/disable transfer on A2 in.
#define MC1_A1IN				0x0001			// enab/disable transfer on A1 in.

// Master Control Register 2 (MC2) constants.
#define MC2_UPLD_DEBI			0x0002			// Upload DEBI.
#define MC2_UPLD_IIC			0x0001			// Upload I2C.
#define MC2_RPSSIG2				0x2000			// RPS signal 2 (not used).
#define MC2_RPSSIG1				0x1000			// RPS signal 1 (DAC RPS busy).
#define MC2_RPSSIG0				0x0800			// RPS signal 0 (ADC RPS busy).

#define MC2_ADC_RPS				MC2_RPSSIG0		// ADC RPS busy.
#define MC2_DAC_RPS				MC2_RPSSIG1		// DAC RPS busy.

// PCI BUS (SAA7146) REGISTER ADDRESS OFFSETS ////////////////////////////////////

#define P_PCI_BT_A		0x004C			// Audio DMA burst/threshold control.
#define P_DEBICFG               0x007C			// DEBI configuration.
#define P_DEBICMD               0x0080			// DEBI command.
#define P_DEBIPAGE              0x0084			// DEBI page.
#define P_DEBIAD                0x0088			// DEBI target address.
#define P_I2CCTRL               0x008C			// I2C control.
#define P_I2CSTAT               0x0090			// I2C status.
#define P_BASEA2_IN				0x00AC			// Audio input 2 base physical DMAbuf address.
#define P_PROTA2_IN				0x00B0			// Audio input 2 physical DMAbuf protection address.
#define P_PAGEA2_IN				0x00B4			// Audio input 2 paging attributes.
#define P_BASEA2_OUT			0x00B8			// Audio output 2 base physical DMAbuf address.
#define P_PROTA2_OUT			0x00BC			// Audio output 2 physical DMAbuf protection address.
#define P_PAGEA2_OUT			0x00C0			// Audio output 2 paging attributes.
#define P_RPSPAGE0              0x00C4			// RPS0 page.
#define P_RPSPAGE1              0x00C8			// RPS1 page.
#define P_RPS0_TOUT				0x00D4			// RPS0 time-out.
#define P_RPS1_TOUT				0x00D8			// RPS1 time-out.
#define P_IER                   0x00DC			// Interrupt enable.
#define P_GPIO                  0x00E0			// General-purpose I/O.
#define P_EC1SSR				0x00E4			// Event counter set 1 source select.
#define P_ECT1R					0x00EC			// Event counter threshold set 1.
#define P_ACON1                 0x00F4			// Audio control 1.
#define P_ACON2                 0x00F8			// Audio control 2.
#define P_MC1                   0x00FC			// Master control 1.
#define P_MC2                   0x0100			// Master control 2.
#define P_RPSADDR0              0x0104			// RPS0 instruction pointer.
#define P_RPSADDR1              0x0108			// RPS1 instruction pointer.
#define P_ISR                   0x010C			// Interrupt status.
#define P_PSR                   0x0110			// Primary status.
#define P_SSR                   0x0114			// Secondary status.
#define P_EC1R					0x0118			// Event counter set 1.
#define P_ADP4					0x0138			// Logical audio DMA pointer of audio input FIFO A2_IN.
#define P_FB_BUFFER1            0x0144			// Audio feedback buffer 1.
#define P_FB_BUFFER2            0x0148			// Audio feedback buffer 2.
#define P_TSL1                  0x0180			// Audio time slot list 1.
#define P_TSL2                  0x01C0			// Audio time slot list 2.

// LOCAL BUS (GATE ARRAY) REGISTER ADDRESS OFFSETS /////////////////////////////////

												// Analog I/O registers:
#define LP_DACPOL				0x0082			//  Write DAC polarity.
#define LP_GSEL					0x0084			//  Write ADC gain.
#define LP_ISEL					0x0086			//  Write ADC channel select.
												// Digital I/O (write only):
#define LP_WRINTSELA			0x0042			//  Write A interrupt enable.
#define LP_WREDGSELA			0x0044			//  Write A edge selection.
#define LP_WRCAPSELA			0x0046			//  Write A capture enable.
#define LP_WRDOUTA				0x0048			//  Write A digital output.
#define LP_WRINTSELB			0x0052			//  Write B interrupt enable.
#define LP_WREDGSELB			0x0054			//  Write B edge selection.
#define LP_WRCAPSELB			0x0056			//  Write B capture enable.
#define LP_WRDOUTB				0x0058			//  Write B digital output.
#define LP_WRINTSELC			0x0062			//  Write C interrupt enable.
#define LP_WREDGSELC			0x0064			//  Write C edge selection.
#define LP_WRCAPSELC			0x0066			//  Write C capture enable.
#define LP_WRDOUTC				0x0068			//  Write C digital output.
												// Digital I/O (read only):
#define LP_RDDINA				0x0040			//  Read digital input.
#define LP_RDCAPFLGA			0x0048			//  Read edges captured.
#define LP_RDINTSELA			0x004A			//  Read interrupt enable register.
#define LP_RDEDGSELA			0x004C			//  Read edge selection register.
#define LP_RDCAPSELA			0x004E			//  Read capture enable register.
#define LP_RDDINB				0x0050			//  Read digital input.
#define LP_RDCAPFLGB			0x0058			//  Read edges captured.
#define LP_RDINTSELB			0x005A			//  Read interrupt enable register.
#define LP_RDEDGSELB			0x005C			//  Read edge selection register.
#define LP_RDCAPSELB			0x005E			//  Read capture enable register.
#define LP_RDDINC				0x0060			//  Read digital input.
#define LP_RDCAPFLGC			0x0068			//  Read edges captured.
#define LP_RDINTSELC			0x006A			//  Read interrupt enable register.
#define LP_RDEDGSELC			0x006C			//  Read edge selection register.
#define LP_RDCAPSELC			0x006E			//  Read capture enable register.
												// Counter Registers (read/write):
#define LP_CR0A					0x0000			//  0A setup register.
#define LP_CR0B					0x0002			//  0B setup register.
#define LP_CR1A					0x0004			//  1A setup register.
#define LP_CR1B					0x0006			//  1B setup register.
#define LP_CR2A					0x0008			//  2A setup register.
#define LP_CR2B					0x000A			//  2B setup register.
												// Counter PreLoad (write) and Latch (read) Registers:
#define	LP_CNTR0ALSW			0x000C			//  0A lsw.
#define	LP_CNTR0AMSW			0x000E			//  0A msw.
#define	LP_CNTR0BLSW			0x0010			//  0B lsw.
#define	LP_CNTR0BMSW			0x0012			//  0B msw.
#define	LP_CNTR1ALSW			0x0014			//  1A lsw.
#define	LP_CNTR1AMSW			0x0016			//  1A msw.
#define	LP_CNTR1BLSW			0x0018			//  1B lsw.
#define	LP_CNTR1BMSW			0x001A			//  1B msw.
#define	LP_CNTR2ALSW			0x001C			//  2A lsw.
#define	LP_CNTR2AMSW			0x001E			//  2A msw.
#define	LP_CNTR2BLSW			0x0020			//  2B lsw.
#define	LP_CNTR2BMSW			0x0022			//  2B msw.
												// Miscellaneous Registers (read/write):
#define LP_MISC1				0x0088			//  Read/write Misc1.
#define LP_WRMISC2				0x0090			//  Write Misc2.
#define LP_RDMISC2				0x0082			//  Read Misc2.

// Bit masks for MISC1 register that are the same for reads and writes.
#define MISC1_WENABLE			0x8000			// enab writes to MISC2 (except Clear Watchdog bit).
#define MISC1_WDISABLE			0x0000			// Disable writes to MISC2.
#define MISC1_EDCAP				0x1000			// enab edge capture on DIO chans specified by LP_WRCAPSELx.
#define MISC1_NOEDCAP			0x0000			// Disable edge capture on specified DIO chans.

// Bit masks for MISC1 register reads.
#define RDMISC1_WDTIMEOUT		0x4000			// Watchdog timer timed out.

// Bit masks for MISC2 register writes.
#define WRMISC2_WDCLEAR			0x8000			// Reset watchdog timer to zero.
#define WRMISC2_CHARGE_ENABLE	0x4000			// enab battery trickle charging.

// Bit masks for MISC2 register that are the same for reads and writes.
#define MISC2_BATT_ENABLE		0x0008			// Backup battery enable.
#define MISC2_WDENABLE			0x0004			// Watchdog timer enable.
#define MISC2_WDPERIOD_MASK		0x0003			// Watchdog interval select mask.

// Bit masks for ACON1 register.
#define A2_RUN					0x40000000				// Run A2 based on TSL2.
#define A1_RUN					0x20000000				// Run A1 based on TSL1.
#define A1_SWAP					0x00200000				// Use big-endian for A1.
#define A2_SWAP					0x00100000				// Use big-endian for A2.
#define WS_MODES				0x00019999				// WS0 = TSL1 trigger input, WS1-WS4 = CS* outputs.

#if PLATFORM == INTEL									// Base ACON1 config: always run A1 based on TSL1.
#define ACON1_BASE				( WS_MODES | A1_RUN )
#elif PLATFORM == MOTOROLA
#define ACON1_BASE				( WS_MODES | A1_RUN | A1_SWAP | A2_SWAP )
#endif

#define ACON1_ADCSTART			ACON1_BASE				// Start ADC: run A1 based on TSL1.
#define ACON1_DACSTART			( ACON1_BASE | A2_RUN )	// Start transmit to DAC: run A2 based on TSL2.
#define ACON1_DACSTOP			ACON1_BASE				// Halt A2.

// Bit masks for ACON2 register.
#define A1_CLKSRC_BCLK1			0x00000000				// A1 bit rate = BCLK1 (ADC).
#define A2_CLKSRC_X1			0x00800000				// A2 bit rate = ACLK/1 (DACs).
#define A2_CLKSRC_X2			0x00C00000				// A2 bit rate = ACLK/2 (DACs).
#define A2_CLKSRC_X4			0x01400000				// A2 bit rate = ACLK/4 (DACs).
#define INVERT_BCLK2			0x00100000				// Invert BCLK2 (DACs).
#define BCLK2_OE				0x00040000				// enab BCLK2 (DACs).
#define ACON2_XORMASK			0x000C0000				// XOR mask for ACON2 active-low bits.

#define ACON2_INIT				( ACON2_XORMASK ^ ( A1_CLKSRC_BCLK1 | A2_CLKSRC_X2 | INVERT_BCLK2 | BCLK2_OE ) )

// Bit masks for timeslot records.
#define WS1		     			0x40000000				// WS output to assert.
#define WS2		     			0x20000000
#define WS3		     			0x10000000
#define WS4		     			0x08000000
#define RSD1					0x01000000				// Shift A1 data in on SD1.
#define SDW_A1					0x00800000				// Store rcv'd char at next char slot of DWORD1 buffer.
#define SIB_A1					0x00400000				// Store rcv'd char at next char slot of FB1 buffer.
#define SF_A1					0x00200000				// Write unsigned long buffer to input FIFO.
														// Select parallel-to-serial converter's data source:
#define XFIFO_0					0x00000000				//   Data fifo byte 0.
#define XFIFO_1					0x00000010				//   Data fifo byte 1.
#define XFIFO_2					0x00000020				//   Data fifo byte 2.
#define XFIFO_3					0x00000030				//   Data fifo byte 3.
#define XFB0					0x00000040				//   FB_BUFFER byte 0.
#define XFB1					0x00000050				//   FB_BUFFER byte 1.
#define XFB2					0x00000060				//   FB_BUFFER byte 2.
#define XFB3					0x00000070				//   FB_BUFFER byte 3.
#define SIB_A2					0x00000200				// Store next dword from A2's input shifter to FB2 buffer.
#define SF_A2					0x00000100				// Store next dword from A2's input shifter to its input fifo.
#define LF_A2					0x00000080				// Load next dword from A2's output fifo into its output dword buffer.
#define XSD2					0x00000008				// Shift data out on SD2.
#define RSD3					0x00001800				// Shift data in on SD3.
#define RSD2					0x00001000				// Shift data in on SD2.
#define LOW_A2					0x00000002				// Drive last SD low for 7 clks, then tri-state.
#define EOS		     			0x00000001				// End of superframe.

#define NUM_TRIMDACS	11						// Number of valid TrimDAC channels.
#define NUM_DACS		4						// Number of valid application DAC channels.

#define NUM_DIOBANKS		3		// Number of DIO groups.
#define NUM_DIOCHANS		48		// Number of DIO channels.
#define NUM_DIOEXTCHANS		40		// Number of extended-capability DIO channels.



//////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////  TYPES  /////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

struct CNTR_OBJ;						//   Counter object type.
struct CORE_OBJ;						//   Core (board) object type.


/////////////////////////////////////////////////////////////////////////////////////
// Counter Channel object.

typedef struct CNTR_OBJ {																// COUNTER OBJECT ------------------------------------------------
																						// Pointers to functions that differ for A and B counters:
	U16		( *GetEnable		)( struct CNTR_OBJ * );									//   Return clock enable.
	U16		( *GetIntSrc		)( struct CNTR_OBJ * );									//   Return interrupt source.
	U16		( *GetLoadTrig		)( struct CNTR_OBJ * );									//   Return preload trigger source.
	U16		( *GetMode			)( struct CNTR_OBJ * );									//   Return standardized operating mode.
	void 	( *PulseIndex		)( struct CNTR_OBJ * );									//   Generate soft index strobe.
	void 	( *SetEnable		)( struct CNTR_OBJ *, U16 enab );						//   Program clock enable.
	void 	( *SetIntSrc		)( struct CNTR_OBJ *, U16 IntSource );					//   Program interrupt source.
	void 	( *SetLoadTrig		)( struct CNTR_OBJ *, U16 Trig );						//   Program preload trigger source.
	void 	( *SetMode			)( struct CNTR_OBJ *, U16 Setup, U16 DisableIntSrc );	//   Program standardized operating mode.
	void 	( *ResetCapFlags	)( struct CNTR_OBJ * );									//   Reset event capture flags.
																						// Protected storage:
	struct CORE_OBJ		*MyBoard;														//   Pointer to parent Cls626 object.
	U16					MyCRA;															//   Address of CRA register.
	U16					MyCRB;															//   Address of CRB register.
	U16					MyLatchLsw;														//   Address of Latch least-significant-word register.
	U16					MyEventBits[4];													//   Bit translations for IntSrc --> RDMISC2.
} CNTR_OBJ;


/////////////////////////////////////////////////////////////////////////////////////
// Core (board) object.

typedef struct CORE_OBJ {				// 626 BOARD'S CORE OBJECT ------------------------

										//   Public storage:
	U32				ErrFlags;			//     Error flags.
	U16				CounterIntEnabs;	//     Counter interrupt enable mask for MISC2 register.

										//   Protected storage that must be initialized by the derived OS interface class:
  // MyAddress renamed to MyBusSlot.  It is NOT the physical address of the board
  // it is a bus and slot number representation
        U32				MyBusSlot;			//     PCI bus & slot number of card.
	PVOID				MyBase;				//     Board base memory address.

										//   Private storage:
	CNTR_OBJ		Counters[6];		//     Counter Channel objects.
	UINT			Board;				//     Logical board number.
	BOOL			IsOpen;				//     Board is open.
	FPTR_ERR		ErrCallback;		//     Pointer to API's error handler callback function.
	U8				AdcItems;			//     Number of items in ADC poll list.
	DMABUF			RPSBuf;				//     DMA buffer used to hold ADC (RPS1) program.
	DMABUF			ANABuf;				//     DMA buffer used to receive ADC data and hold DAC data.
	PVOID				pDacWBuf;			//     Pointer to logical adrs of DMA buffer used to hold DAC data.
	U16				Dacpol;				//     Image of DAC polarity register.
	S16				DacSetpoint[4];		//     Images of DAC setpoints.
	U8				TrimSetpoint[NUM_TRIMDACS];	//     Images of TrimDAC setpoints.
	U16				DOutImage[NUM_DIOBANKS];		//     Images of DIO output registers.
	U16				ChargeEnabled;		//     Image of MISC2 Battery Charge Enabled (0 or WRMISC2_CHARGE_ENABLE).
	U16				WDInterval;			//     Image of MISC2 watchdog interval control bits.
	U32				I2CAdrs;			//     I2C device address for onboard EEPROM (board rev dependent).
} CORE_OBJ;


//////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////  STORAGE  ///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

static struct CORE_OBJ	co[MAX_BOARDS];		// The 626 board core objects.
static UINT				nboards = 0;		// Number of core objects declared by the module layer.


//////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////  FUNCTION PROTOTYPES  ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

static void		DummyErrHandler	( DWORD ErrMask );

static void *		GetAddress	( struct CORE_OBJ *c );

static void		CountersInit	( struct CORE_OBJ *c );
static void		CountersCreate	( struct CORE_OBJ *c );
static void		DEBIinit		( struct CORE_OBJ *c  );
static U16		DEBIread		( struct CORE_OBJ *c, U16 addr );
static void		DeclareError	( struct CORE_OBJ *c, U32 ErrMask );
static void		DIOInit			( struct CORE_OBJ *c );
static void		I2CInit			( struct CORE_OBJ *c );
static void		InitADC			( struct CORE_OBJ *c );
static void		InitDAC			( struct CORE_OBJ *c );
static void		InterruptMask	( struct CORE_OBJ *c );
static void		LoadTrimDACs	( struct CORE_OBJ *c );
static void		SendDAC			( struct CORE_OBJ *c, U32 val );
static void		SetDAC			( struct CORE_OBJ *c, U16 chan, S16 dacdata );
static void		WriteMISC2		( struct CORE_OBJ *c, U16 NewImage );


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////  FUNCTION MACROS  /////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

// address size is 4 bytes or unsigned long
#define WRITE_RPS(addr, val) { S626MOD_WriteMem( addr,val); addr =(PVOID) ( (unsigned char *) addr + 4);}

// Write/Read U32 to/from SAA7146 device register.
//DA: bit systems compatibility.  Pointers are 64bit.
#define WR7146( REGADRS, VAL )	S626MOD_WriteMem( (PVOID)((unsigned char *) c->MyBase + (long) (REGADRS) ), (VAL) )
#define RR7146( REGADRS )	S626MOD_ReadMem ( (PVOID)((unsigned char *) c->MyBase + (long) (REGADRS) ) )


// enab/disable a function or test status bit(s) that are accessed through Main Control Registers 1 or 2.
#define MC_ENABLE( REGADRS, CTRLWORD )			WR7146( ( REGADRS ), ( (U32)( CTRLWORD ) << 16 ) | (U32)( CTRLWORD ) )
#define MC_DISABLE( REGADRS, CTRLWORD )			WR7146( ( REGADRS ), (U32)( CTRLWORD ) << 16 )
#define MC_TEST( REGADRS, CTRLWORD )			( ( RR7146( REGADRS ) & CTRLWORD ) != 0 )

// Write a time slot control record to TSL2.
#define VECTPORT( VECTNUM )						( P_TSL2 + ( (VECTNUM) << 2 ) )
#define SETVECT( VECTNUM, VECTVAL )				WR7146( VECTPORT( VECTNUM ), (VECTVAL) )

// Validate a function argument by comparing to maximum legal value.
#define ValidateParm( PARM, MAXVAL )			if ( ( PARM ) >= ( MAXVAL ) ) { DeclareError( c, ERR_ILLEGAL_PARM ); return; }
#define ValidateParmRtnval( PARM, MAXVAL )		if ( ( PARM ) >= ( MAXVAL ) ) { DeclareError( c, ERR_ILLEGAL_PARM ); return 0; }


//////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////  STARTUP & SHUTDOWN FUNCTIONS  //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Map logical board number into pointer to board's core object.
// Returns zero if board number is not legal.

static struct CORE_OBJ *GetBoardObject( HBD bd )
{
	// Cache pointer to board's core object, if bd is valid.
	return ( bd < nboards ) ? &co[bd] : (struct CORE_OBJ *)0;
}



/////////////////////////////////////////////////////
// Core object set constructor/destructor.

UINT S626CORE_CoreOpen( HBD bd, UINT num_boards )
{
	struct CORE_OBJ *c;

	// Abort if declared board count exceeds max allowed value.
	if ( num_boards > MAX_BOARDS )
		return FALSE;

	// Save the declared board count.
	nboards = num_boards;

	// Abort if invalid parameters specified.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return FALSE;

	// Initialize object storage members.
	c->Board		= bd;				// Logical board number in CORE <= Board number in MOD: [0, 1, ..., (MAX_BOARDS-1)].
	c->IsOpen		= FALSE;			// Board is closed.
	c->MyBusSlot	= 0;				// Reset board physical address.
	c->ErrFlags		= 0;				// Clear all error flags.
	c->ErrCallback	= DummyErrHandler;	// Init pointer to error handler function.

	// Create counter objects.
	CountersCreate( c );

	// Indicate success.
	return TRUE;
}

void S626CORE_CoreClose( HBD bd )
{
	//struct CORE_OBJ *c = &co[board];
	struct CORE_OBJ *c;

	// Abort if invalid parameters specified.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return;
	
	// Close any boards that are open.
	c->IsOpen		= FALSE;			// Board is closed.

}

////////////////////////////////////////////////////////////////////////////////////
// OPEN BOARD
// IsBoardRevA is a boolean that indicates whether the board is RevA.
////////////////////////////////////////////////////////////////////////////////////

void S626CORE_BoardOpen( HBD bd, U8 IsBoardRevA )
{
	struct CORE_OBJ	*c;
	U16				chan;

	// Abort if invalid parameters specified.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return;

	// If board is closed ...
	if ( !c->IsOpen )
	{
		// Start thread-safe execution.
		S626MOD_CriticalBegin( c->Board );

		// Disable interrupts.
		InterruptMask( c );
		// Invoke SAA7146 soft reset.
		WR7146( P_MC1, MC1_SOFT_RESET );

		// Allocate DMA buffers using S626MOD_AllocDMAB(), a pure virtual function provided by the OS Interface class.
		// Each call to S626MOD_AllocDMAB() allocates physically contiguous buffer space at least as large as the
		// requested size.  The buffer must begin on a page boundary, and pages must contain at least 4KB
		// of contiguous memory.  This class (and the S626MOD_AllocDMAB() function) must be modified if the target
		// OS can not provide the minimum 4KB physical page size.  S626MOD_AllocDMAB() sets the "allocation error"
		// flag in ErrFlags if any buffer allocation fails.
		S626MOD_AllocDMAB( bd, DMABUF_SIZE, &c->ANABuf );		// Allocate buffer to receive ADC data.
		S626MOD_AllocDMAB( bd, DMABUF_SIZE, &c->RPSBuf );		// Allocate buffer for RPS1 commands that will control the ADC.

		// If DMA buffers were successfully allocated.
		if ( c->ErrFlags == 0 )
		{
			// enab DEBI and audio pins, enable I2C interface.
			MC_ENABLE( P_MC1, MC1_DEBI | MC1_AUDIO | MC1_I2C );

			// Init DEBI functional configuration used for all DEBI operations.
			DEBIinit( c );

			// Init GPIO so that ADC Start* is negated.
			WR7146( P_GPIO, GPIO_BASE | GPIO1_HI );

			// VERSION 2.01 CHANGE: REV A & B BOARDS NOW SUPPORTED BY DYNAMIC EEPROM ADDRESS SELECTION.
			// Initialize the I2C interface, which is used to access the onboard serial EEPROM.  The EEPROM's I2C
			// DeviceAddress is hardwired to a value that is dependent on the 626 board revision.  On all board
			// revisions, the EEPROM stores TrimDAC calibration constants for analog I/O.  On RevB and higher
			// boards, the DeviceAddress is hardwired to 0 to enable the EEPROM to also store the PCI SubVendorID
			// and SubDeviceID; this is the address at which the SAA7146 expects a configuration EEPROM to reside.
			// On RevA boards, the EEPROM device address, which is hardwired to 4, prevents the SAA7146 from
			// retrieving PCI sub-IDs, so the SAA7146 uses its built-in default values, instead.
			c->I2CAdrs = IsBoardRevA ? 0xA8 : 0xA0;			// Set I2C EEPROM DeviceType (0xA0) and DeviceAddress<<1.
			I2CInit( c );								// Initialize I2C interface.

			// Init audio interface functional attributes:  set DAC/ADC serial clock rates, invert DAC
			// serial clock so that DAC data setup times are satisfied, enable DAC serial clock out.
			WR7146( P_ACON2, ACON2_INIT );

			// Init the ADC interface.
			InitADC( c );

			// Init the DAC interface.
			InitDAC( c );

			// Init Trim DACs to calibrated values.  Do it twice because the SAA7146 audio channel does
			// not always reset properly and sometimes causes the first few TrimDAC writes to malfunction.
			LoadTrimDACs( c );
			LoadTrimDACs( c );		// Insurance.

			/////////////////////////////////////////////////////////////////////////////////////////////
			// Manually init all gate array hardware in case this is a soft reset (we have no way of
			// determining whether this is a warm or cold start).  This is necessary because the gate
			// array will reset only in response to a PCI hard reset; there is no soft reset function.

			// Init all DAC outputs to 0V and init all DAC setpoint and polarity images.
			for ( chan = 0; chan < 4; chan++)
				SetDAC( c, chan, 0 );

			// Init image of WRMISC2 Battery Charger Enabled control bit.  This image is used when the
			// state of the charger control bit, which has no direct hardware readback mechanism, is queried.
			c->ChargeEnabled = 0;

			// Init image of watchdog timer interval in WRMISC2.  This image maintains the value of the
			// control bits of MISC2 are continuously reset to zero as long as the WD timer is disabled.
			c->WDInterval = 0;

			// Init Counter Interrupt enab mask for RDMISC2.  This mask is applied against MISC2 when
			// testing to determine which timer events are requesting interrupt service.
			c->CounterIntEnabs = 0;

			// Init counter objects.
			CountersInit( c );

			// Without modifying the state of the Battery Backup enab, disable the watchdog timer, set
			// DIO channels 0-5 to operate in the standard DIO (vs. counter overflow) mode, disable
			// the battery charger, and reset the watchdog interval selector to zero.
			WriteMISC2( c, (U16)( DEBIread( c, LP_RDMISC2 ) & MISC2_BATT_ENABLE ) );

			// Initialize the digital I/O subsystem.
			DIOInit( c );

			// Indicate board is open.
			c->IsOpen = TRUE;
		}

		// End of thread-safe segment.
		S626MOD_CriticalEnd( c->Board );
	}
}

////////////////////////////////////////////////////////////////////////////
// CLOSE BOARD.

void S626CORE_BoardClose( HBD bd )
{
	struct CORE_OBJ *c;

	// Abort if invalid parameters specified.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return;		// If board number is not valid ...

	// If board is open ...
	if ( c->IsOpen )
	{
		// Start thread-safe execution.
		S626MOD_CriticalBegin( c->Board );

		// Disable interrupts.
		InterruptMask( c );

		// Disable the watchdog timer and battery charger.
		WriteMISC2( c, 0 );

		// Close all interfaces on 7146 device.
		WR7146( P_MC1, MC1_SHUTDOWN );
		WR7146( P_ACON1, ACON1_BASE );

		// Release memory used by DMA buffers.
		S626MOD_CloseDMAB( bd, &c->RPSBuf );
		S626MOD_CloseDMAB( bd, &c->ANABuf );

		// Indicate board is close.
		c->IsOpen = FALSE;

		// End of thread-safe segment.
		S626MOD_CriticalEnd( c->Board );
	}

}

//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////  STATUS AND ERROR FUNCTIONS  ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////
// Set/return board PCI bus/slot numbers.

U32 S626CORE_GetAddress( HBD bd )
{
	struct CORE_OBJ	*c;
	U32 rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		if ( c->IsOpen )
		  rtnval = c->MyBusSlot;
	}

	return rtnval;
}

void S626CORE_SetAddress( HBD bd, U32 adrs )
{
	struct CORE_OBJ	*c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Set address.
		c->MyBusSlot = adrs;
	}
	return;
}

/////////////////////////////////////////////////////////
// Set/return board base address for register access.

PVOID S626CORE_GetBase( HBD bd )
{
	struct CORE_OBJ	*c;
	PVOID rtnval = NULL;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		if ( c->IsOpen )
		  rtnval = c->MyBase;
	}

	return rtnval;
}

void S626CORE_SetBase( HBD bd, PVOID base )
{
	struct CORE_OBJ	*c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Set base address.
		c->MyBase = base;
	}
}

/////////////////////////////////////////////////////////////
// Enable application's error handler callback function.

void S626CORE_SetErrCallback( HBD bd, FPTR_ERR Callback )
{
	struct CORE_OBJ	*c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// If board is open ...
		if ( c->IsOpen )
			c->ErrCallback = Callback;
	}
}

/////////////////////////////////////////////////////////////
// Return error flags and clear any resettable error flags.

U32 S626CORE_GetErrors( HBD bd )
{
	struct CORE_OBJ	*c;
	U32				rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// If board is open ...
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			// Cache return value.
			rtnval = c->ErrFlags;

			// Clear any resettable error flags.
			c->ErrFlags &= 0x0000FFFF;

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );

		} else {

			// Cache return value.
			rtnval = c->ErrFlags;

			// Clear any error flags other than ERR_OPEN since it is not opened.
			c->ErrFlags &= 0x00000001;
		}
	}

	// return original error flags.
	return rtnval;
}

/////////////////////////////////////////////////////////////
// Set specified error flags, or reset all flags if zero.

void S626CORE_SetErrors( HBD bd, U32 NewFlags )
{
	struct CORE_OBJ	*c;

	// Set specified error flags if board number is valid.
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		if ( NewFlags )
			c->ErrFlags |= NewFlags;
		else
			c->ErrFlags = 0;
	}
}

/////////////////////////////////////////////////////////////
// Set one or more error flags.

static void DeclareError( struct CORE_OBJ *c, U32 ErrMask )
{
	// Start critical segment.
	//S626MOD_CriticalBegin( c->Board );	// done at upper level.

	// Set error flag bits as specified by ErrMask.
	c->ErrFlags |= ErrMask;

	// Call the application's error handler.  If the application has not yet declared an error
	// handler via SetErrCallback(), the DummyErrHandler() function is executed by default.
	c->ErrCallback( c->ErrFlags );

	// End critical segment.
	//S626MOD_CriticalEnd( c->Board );		// done at upper level.
}

/////////////////////////////////////////////////////////////////////////////////////
// This dummy error handler is substituted for the application's callback function
// if the application has not yet called SetErrCallback() to declare a callback.
// Note: this function is not a class member because it is expected to be cdecl.

static void DummyErrHandler( DWORD ErrMask )	{}

//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////  EEPROM ACCESS FUNCTIONS  //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

// I2C configuration constants.
#define I2C_CLKSEL			0x0400								// I2C bit rate = PCIclk/480 = 68.75 KHz.
#define I2C_BITRATE			68.75								// I2C bus data bit rate (determined by I2C_CLKSEL) in KHz.
#define I2C_WRTIME			15.0								// Worst case time,in msec, for EEPROM internal write op.

// I2C manifest constants.
#define I2C_RETRIES			( I2C_WRTIME * I2C_BITRATE / 9.0 )	// Max retries to wait for EEPROM write.
#define I2C_ERR				0x0002								// I2C control/status flag ERROR.
#define I2C_BUSY			0x0001								// I2C control/status flag BUSY.
#define I2C_ABORT			0x0080								// I2C status flag ABORT.
#define I2C_ATTRSTART		0x3									// I2C attribute START.
#define I2C_ATTRCONT		0x2									// I2C attribute CONT.
#define I2C_ATTRSTOP		0x1									// I2C attribute STOP.
#define I2C_ATTRNOP			0x0									// I2C attribute NOP.
#define I2CR				( c->I2CAdrs | 1 )				// I2C read command  | EEPROM address.
#define I2CW				( c->I2CAdrs )					// I2C write command | EEPROM address.

// Code macros used for constructing I2C command bytes.
#define I2C_B2(ATTR,VAL)	( ( (ATTR) << 6 ) | ( (VAL) << 24 ) )
#define I2C_B1(ATTR,VAL)	( ( (ATTR) << 4 ) | ( (VAL) << 16 ) )
#define I2C_B0(ATTR,VAL)	( ( (ATTR) << 2 ) | ( (VAL) <<  8 ) )

///////////////////////////////////////////////////////////////////////////////////
// Private helper function: handshake value to EEPROM.  Returns 0 if successful.

static U32 I2Chandshake( struct CORE_OBJ *c, U32 val )
{
	// Write I2C command to I2C Transfer Control shadow register.
	WR7146( P_I2CCTRL, val );

	// Upload I2C shadow registers into working registers and wait for upload confirmation.
	MC_ENABLE( P_MC2, MC2_UPLD_IIC );

	while ( !MC_TEST( P_MC2, MC2_UPLD_IIC ) );

	// Wait until I2C bus transfer is finished or an error occurs.
	while ( ( RR7146(P_I2CCTRL) & ( I2C_BUSY | I2C_ERR ) ) == I2C_BUSY );
	
	// Return non-zero if I2C error occured.
	return RR7146(P_I2CCTRL) & I2C_ERR;
}

///////////////////////////////////////////
// Read U8 from EEPROM.

static U8 I2Cread( struct CORE_OBJ *c, U8 addr )
{
	U8 rtnval;

	// Send EEPROM target address.
	if ( I2Chandshake( c,
		  I2C_B2( I2C_ATTRSTART, I2CW )		// Byte2 = I2C command: write to I2C EEPROM device.
		| I2C_B1( I2C_ATTRSTOP,  addr )		// Byte1 = EEPROM internal target address.
		| I2C_B0( I2C_ATTRNOP,   0    ) ) )	// Byte0 = Not sent.
	{
		// Abort function and declare error if handshake failed.
		DeclareError( c, ERR_I2C );
		return 0;
	}

	// Execute EEPROM read.
	if ( I2Chandshake( c,
		  I2C_B2( I2C_ATTRSTART, I2CR )		// Byte2 = I2C command: read from I2C EEPROM device.
		| I2C_B1( I2C_ATTRSTOP,  0    )		// Byte1 receives U8 from EEPROM.
		| I2C_B0( I2C_ATTRNOP,   0    ) ) )	// Byte0 = Not sent.
	{
		// Abort function and declare error if handshake failed.
		DeclareError( c, ERR_I2C );
		return 0;
	}

	// Return copy of EEPROM value.
	rtnval = (U8)( RR7146(P_I2CCTRL) >> 16 );
	return rtnval;
}

U8 S626CORE_I2Cread( HBD bd, U8 addr )
{
	struct CORE_OBJ *c;
	U8				retval;

	// Abort if board number is not valid.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// If board is open ...
	if ( c->IsOpen )
	{
		// Start critical segment.
		S626MOD_CriticalBegin( c->Board );

		// Execute I2C read.
		retval = I2Cread( c, addr );

		// End critical segment.
		S626MOD_CriticalEnd( c->Board );

	} else {
		retval = 0;
	}

	return retval;

}

////////////////////////////////////////////
// Write U8 to EEPROM.

static void I2Cwrite( struct CORE_OBJ *c, U8 addr, U8 val )
{
	U32 retries;

	// Send EEPROM target address and new data value and invoke internal EEPROM write operation.
	if ( I2Chandshake( c,
		  I2C_B2( I2C_ATTRSTART, I2CW )		// Byte2 = I2C command: write to I2C EEPROM device.
		| I2C_B1( I2C_ATTRCONT,  addr )		// Byte1 = EEPROM internal target address.
		| I2C_B0( I2C_ATTRSTOP,  val  ) ) )	// Byte0 = New EEPROM data value.
	{
		// Abort function and declare error if handshake failed.
		DeclareError( c, ERR_I2C );
		return;
	}

	// Wait for the EEPROM to finish its internal write operation.  This is done by polling with
	// dummy write commands until the EEPROM responds by ACKing the I2C commands being sent to it.
	// A retry counter is used so that we won't hang if there is an I2C hardware fault.
	for ( retries = 0; retries < (U32)I2C_RETRIES; retries++ )
	{
		// Halt any pending I2C operations and clear any I2C errors.
		I2CInit( c );

		// Attempt to handshake a command to the EEPROM.  The handshake will fail
		// if the EEPROM is still executing its internal write operation.
		if ( !I2Chandshake( c,
			  I2C_B2( I2C_ATTRSTART, I2CW )			// Byte2 = I2C command: write to I2C EEPROM device.
			| I2C_B1( I2C_ATTRSTOP,  addr )			// Byte1 = EEPROM internal target address.
			| I2C_B0( I2C_ATTRNOP,   0    ) ) )		// Byte0 = Not sent.
		{
			return;			// Exit if EEPROM write operation is finished.
		}
	}

	// Abort function and declare error if handshake failed.
	DeclareError( c, ERR_I2C );
}

void S626CORE_I2Cwrite( HBD bd, U8 addr, U8 val )
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Execute I2C write if board is open.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			// Execute I2C write.
			I2Cwrite( c, addr, val );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}

//////////////////////////////////////////////////////////
// Private helper function: initialize I2C interface.

static void I2CInit( struct CORE_OBJ *c )
{
	register int i;

	// Issue an I2C ABORT command to halt any I2C operation in progress and reset BUSY flag.
	WR7146( P_I2CSTAT, I2C_CLKSEL | I2C_ABORT );			// Write I2C control: abort any I2C activity.
	MC_ENABLE( P_MC2, MC2_UPLD_IIC );						// Invoke command upload
	while ( ( RR7146(P_MC2) & MC2_UPLD_IIC ) == 0 );		//   and wait for upload to complete.
	// Per SAA7146 data sheet, write to STATUS reg twice to reset all I2C error flags.
	for ( i = 0; i < 2; i++ )
	{
		WR7146( P_I2CSTAT, I2C_CLKSEL );					// Write I2C control: reset error flags.
		MC_ENABLE( P_MC2, MC2_UPLD_IIC );					// Invoke command upload
		while ( !MC_TEST( P_MC2, MC2_UPLD_IIC ) );			//   and wait for upload to complete.
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////  DEBI (GATE ARRAY REGISTER ACCESS) FUNCTIONS  //////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// DEBI command constants.
#define DEBI_CMD_SIZE16			( 2 << 17 )			// Transfer size is always 2 bytes.
#define DEBI_CMD_READ			0x00010000			// Read operation.
#define DEBI_CMD_WRITE			0x00000000			// Write operation.

#define DEBI_CMD_RDWORD			( DEBI_CMD_READ  | DEBI_CMD_SIZE16 )	// Read immediate 2 bytes.
#define DEBI_CMD_WRWORD			( DEBI_CMD_WRITE | DEBI_CMD_SIZE16 )	// Write immediate 2 bytes.

// DEBI configuration constants.
#define DEBI_CFG_XIRQ_EN		0x80000000			// enab external interrupt on GPIO3.
#define DEBI_CFG_XRESUME		0x40000000			// Resume block transfer when XIRQ deasserted.
#define DEBI_CFG_FAST			0x10000000			// Fast mode enable.
													// 4-bit field that specifies DEBI timeout value in PCI clock cycles:
#define DEBI_CFG_TOUT_BIT		22					//   Finish DEBI cycle after this many clocks.
													// 2-bit field that specifies Endian byte lane steering:
#define DEBI_CFG_SWAP_NONE		0x00000000			//   Straight - don't swap any bytes (Intel).
#define DEBI_CFG_SWAP_2			0x00100000			//   2-byte swap (Motorola).
#define DEBI_CFG_SWAP_4			0x00200000			//   4-byte swap.
#define DEBI_CFG_SLAVE16		0x00080000			// Slave is able to serve 16-bit cycles.
#define DEBI_CFG_INC			0x00040000			// enab address increment for block transfers.
#define DEBI_CFG_INTEL			0x00020000			// Intel style local bus.
#define DEBI_CFG_TIMEROFF		0x00010000			// Disable timer.

#if PLATFORM == INTEL

#define DEBI_TOUT				7					// Wait 7 PCI clocks (212 ns) before polling RDY.
#define DEBI_SWAP				DEBI_CFG_SWAP_NONE	// Intel byte lane steering (pass through all byte lanes).

#elif PLATFORM == MOTOROLA

#define DEBI_TOUT				15					// Wait 15 PCI clocks (454 ns) maximum before timing out.
#define DEBI_SWAP				DEBI_CFG_SWAP_2		// Motorola byte lane steering.

#endif

// DEBI page table constants.
#define DEBI_PAGE_DISABLE		0x00000000			// Paging disable.

////////////////////////////////////////////////////////////////////////////////////
// Initialize the DEBI interface for all transfers.

static void DEBIinit( struct CORE_OBJ *c  )
{
	// Configure DEBI operating mode.
	WR7146( P_DEBICFG,  DEBI_CFG_SLAVE16			// Local bus is 16 bits wide.
		| ( DEBI_TOUT << DEBI_CFG_TOUT_BIT )		// Declare DEBI transfer timeout interval.
		| DEBI_SWAP									// Set up byte lane steering.
		| DEBI_CFG_INTEL );							// Intel-compatible local bus (DEBI never times out).

	// Paging is disabled.
	WR7146( P_DEBIPAGE, DEBI_PAGE_DISABLE );		// Disable MMU paging.
}

////////////////////////////////////////////////////////////////////////////////////
// Execute a DEBI transfer.  This must be called from within a critical section.

static void DEBItransfer( struct CORE_OBJ *c )
{
	// Initiate upload of shadow RAM to DEBI control register.
	MC_ENABLE( P_MC2, MC2_UPLD_DEBI );
	// Wait for completion of upload from shadow RAM to DEBI control register.
	while ( !MC_TEST( P_MC2, MC2_UPLD_DEBI ) );
	// Wait until DEBI transfer is done.
	while ( RR7146(P_PSR) & PSR_DEBI_S );
}

/////////////////////////////////////////////////////////////////
// Return a value from a gate array register.

static U16 DEBIread( struct CORE_OBJ *c, U16 addr )
{
	U16 retval;

	// Start critical segment.
	//S626MOD_CriticalBegin( c->Board );	// done at upper level.

	// Set up DEBI control register value in shadow RAM.
	WR7146( P_DEBICMD, DEBI_CMD_RDWORD | addr );

	// Execute the DEBI transfer.
	DEBItransfer( c );

	// Fetch target register value.
	retval = (U16)RR7146( P_DEBIAD );

	// End critical segment.
	//S626MOD_CriticalEnd( c->Board );		// done at upper level.

	// Return register value.
	return retval;
}

U16 S626CORE_DEBIread( HBD bd, U16 addr )
{
	struct CORE_OBJ *c;
	U16				retval;

	// Abort if board number is not valid.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Execute DEBI read if board is open.
	if ( c->IsOpen )
	{
		// Start critical segment.
		S626MOD_CriticalBegin( c->Board );

		retval = DEBIread( c, addr );

		// End critical segment.
		S626MOD_CriticalEnd( c->Board );
	} else {
		retval = 0;
	}

	return retval;

}

/////////////////////////////////////////////////////////////////////////
// Write a value to a gate array register.

static void DEBIwrite( struct CORE_OBJ *c, U16 addr, U16 wdata )
{
	// Start critical segment.
	//S626MOD_CriticalBegin( c->Board );	// done at upper level.

	// Set up DEBI control register value in shadow RAM.
	WR7146( P_DEBICMD, DEBI_CMD_WRWORD | addr );
	WR7146( P_DEBIAD,  wdata );

	// Execute the DEBI transfer.
	DEBItransfer( c );

	// End critical segment.
	//S626MOD_CriticalEnd( c->Board );		// done at upper level.
}

void S626CORE_DEBIwrite( HBD bd, U16 addr, U16 wdata )
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Execute DEBI write if board is open.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			DEBIwrite( c, addr, wdata );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////
// Replace the specified bits in a gate array register.  Imports:  mask specifies bits
// that are to be preserved, wdata is new value to be or'd with the masked original.

static void DEBIreplace( struct CORE_OBJ *c, U16 addr, U16 mask, U16 wdata )
{
	// Start critical segment.
	//S626MOD_CriticalBegin( c->Board );		// done at upper level.
	// Copy target gate array register into P_DEBIAD register.
	WR7146( P_DEBICMD, DEBI_CMD_RDWORD | addr );							// Set up DEBI control reg value in shadow RAM.
	DEBItransfer( c );													// Execute the DEBI Read transfer.
	// Write back the modified image.
	WR7146( P_DEBICMD, DEBI_CMD_WRWORD | addr );							// Set up DEBI control reg value in shadow RAM.
	WR7146( P_DEBIAD, wdata | ( (U16)RR7146( P_DEBIAD ) & mask ) );			// Modify the register image.
	DEBItransfer( c );													// Execute the DEBI Write transfer.
	// End critical segment.
	//S626MOD_CriticalEnd( c->Board );		// done at upper level.
}

//////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////  INTERRUPT FUNCTIONS  ///////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
// IntActions is the interrupt handler action list for the OS interface class.  This array,
// which is not a member of Cls626, returns a list of Write actions that must take place at the
// beginning of the ISR.  It is used by the OS interface class to set up all kernel-mode driver
// accesses that must occur at the beginning of interrupt service.  Whether to use IntActions
// versus InterruptEnable() depends on the the target OS.  InterruptEnable() is used only if the
// target OS supports direct register access during initial interrupt handling.  If the OS does
// not support this, IntActions must be used instead of InterruptEnable().

// Actions used to negate IRQ at start of hardware interrupt service:

#define	ACT_READ	0
#define ACT_WRITE	1

INT_ACTIONS S626CORE_IntActions = {
	IRQ_MASK,									/* AND mask for IRQ bit in status register.	*/
	0,											/* XOR mask for IRQ bit in status register.	*/

	{	1,										/* 1 action used to retrieve IRQ status.	*/
		{										/*   Actions:								*/
			{ ACT_READ,  P_ISR, 0		 }		/*     Fetch IRQ Pending status.  			*/
		}
	},

	{	2,										/* 2 actions used to negate IRQ.			*/
		{										/*   Actions:								*/
			{ ACT_WRITE, P_IER, 0        },		/*     Disable interrupt.					*/
			{ ACT_WRITE, P_ISR, IRQ_MASK }		/*     Clear IRQ.							*/
		}
	}
};

/////////////////////////////////////////////////////////////////
// enab/disable master hardware interrupt.

void S626CORE_InterruptEnableSet( HBD bd, U16 enab )
{
	struct CORE_OBJ *c;
	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// If board is open ...
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			if ( enab ) {
				WR7146( P_IER, IRQ_MASK );		// enab master interrupt.
			}
			else {
				InterruptMask( c );				// Disable master interrupt.
			}

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}

/////////////////////////////////////////////////////////////////
// Return state of master hardware interrupt's enable.

U16 S626CORE_InterruptEnableGet( HBD bd )
{
	struct CORE_OBJ *c;
	U16 rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Return non-zero if master interrupt is enabled.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			rtnval = ( ( RR7146( P_IER ) & IRQ_MASK ) != 0 );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}

	return rtnval;
}

/////////////////////////////////////////////////////////////////
// Disable master hardware interrupt and clear any pending IRQ.

static void InterruptMask( struct CORE_OBJ *c )
{
	//S626MOD_CriticalBegin( c->Board );	// Start critical segment.	(done at upper level)
	WR7146( P_IER, 0 );						// Disable master interrupt.
	WR7146( P_ISR, IRQ_MASK );				// Clear board's master IRQ status flag.
	//S626MOD_CriticalEnd( c->Board );		// End critical segment.	(done at upper level)
}

void S626CORE_InterruptMask( HBD bd )
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Mask interrupt if board is open.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			InterruptMask( c );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Return non-zero value if the master interrupt service request flag is asserted, independent
// of whether the master IRQ is enabled.

U16 S626CORE_InterruptPending( HBD bd )
{
	struct CORE_OBJ *c;
	U16 rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Return non-zero if GPIO3 (which is the gate-array's active-high Interrupt Service Request signal) is asserted.
		if ( c->IsOpen )
			rtnval = ( ( RR7146( P_PSR ) & IRQ_MASK ) != 0 );
	}

	return rtnval;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return interrupt request status from all IRQ sources.  An array of four words is
// returned.  The first three words indicate IRQs on the three DIO channel groups:  a
// "1" in a bit position indicates that an IRQ is pending on the corresponding channel.
// The fourth U16 indicates IRQs on the six counter channels in the bit order defined
// by the RDMISC2 register:  a "1" in a bit position indicated a pending IRQ for the
// associated overflow or index event.  This function provides a high-performance
// mechanism for interrupt handlers to determine IRQ status of DIO/counter channels.

void S626CORE_InterruptStatus( HBD bd, U16 *IntFlags )
{
	struct CORE_OBJ *c;

	// Zero out the flags in case the board is not open.
	*IntFlags = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// If board is open ...
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			// Get all DIO IRQs.  These are derived by masking DIO events captures against the DIO interrupt
			// enables that are stored in the readable interrupt enable registers.
			*IntFlags++ = DEBIread( c, LP_RDCAPFLGA ) & DEBIread( c, LP_RDINTSELA );
			*IntFlags++ = DEBIread( c, LP_RDCAPFLGB ) & DEBIread( c, LP_RDINTSELB );
			*IntFlags++ = DEBIread( c, LP_RDCAPFLGC ) & DEBIread( c, LP_RDINTSELC );

			// Get all counter IRQs.  These are derived by masking counter event captures (in the RDMISC2
			// register) against the interrupt enables stored in the CounterIntEnabs image.
			*IntFlags = DEBIread( c, LP_RDMISC2 ) & c->CounterIntEnabs;

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////  ADC FUNCTIONS  ////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

// SAA7146 BUG WORKAROUND
// According to Philips Errata Info, Ref 1.1, dated 16 Jan 1998, the RPS STREG command must
// specify a register address offset that is 4 less than the target address offset.

#define BUGFIX_STREG(REGADRS)	( REGADRS - 4 )

////////////////////////////////////////////////////////////////////////////////////
// Execute ADC poll list scan and return ADC data in application pdata[] buffer.

static void StartADC( struct CORE_OBJ *c )
{
	// Start ADC critical segment.
	S626MOD_CriticalBeginAdc( c->Board );

	// Trigger ADC scan loop start by setting RPS Signal 0.
	MC_ENABLE( P_MC2, MC2_ADC_RPS );
}

void S626CORE_StartADC( HBD bd )
{
	struct CORE_OBJ	*c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Start ADC if board is open.
		if ( c->IsOpen )
			StartADC( c );
	}
}

static void WaitDoneADC( struct CORE_OBJ *c, U16 *pdata )
{
	U8	i;
	U32     *readaddr;
	// Wait until ADC scan loop is finished (RPS Signal 0 reset).
	while ( MC_TEST( P_MC2, MC2_ADC_RPS ) );

	//DA: 3/3/2007 comment changed.  skip first U32 not U16
	// Init ptr to DMA buffer that holds new ADC data.  We skip the first U32 in the buffer
	// because it contains junk data from the final ADC of the previous poll list scan.
	readaddr = (U32 *)c->ANABuf.LogicalBase + 1;

	// Convert ADC data to 16-bit integer values and copy to application buffer.
	for ( i = 0; i < c->AdcItems; i++ ) {
		*pdata++ = (U16)( S626MOD_ReadMem( readaddr++ ) >> 16 );
	}

	// End ADC critical segment.
	S626MOD_CriticalEndAdc( c->Board );
}

void S626CORE_WaitDoneADC( HBD bd, U16 *pdata )
{
	struct CORE_OBJ	*c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Wait for ADC done if board is open.
		if ( c->IsOpen )
			WaitDoneADC( c, pdata );
	}
}

static void ReadADC( struct CORE_OBJ *c, U16 *pdata )
{
	// Initiate ADC scan.
	StartADC( c );

	// Wait for ADC scan to finish.
	WaitDoneADC( c, pdata );
}

void S626CORE_ReadADC( HBD bd, U16 *pdata )
{
	struct CORE_OBJ	*c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Execute ADC read if board is open.
		if ( c->IsOpen )
			ReadADC( c, pdata );
	}
}

////////////////////////////////////////////////////////////////////////////
// Prepare for execution of the ADC poll list.

#define GSEL_BIPOLAR5V		0x00F0		// LP_GSEL setting for 5V bipolar range.
#define GSEL_BIPOLAR10V		0x00A0		// LP_GSEL setting for 10V bipolar range.



static void ResetADC( struct CORE_OBJ *c, U8 *ppl )
{
  //        register U32	*pRPS;
	PVOID                           pRPS;
	U32                             PhysicalBase;
	U32				JmpAdrs;
	U16				i;
	U16				n;
	U32				LocalPPL;

	// Stop RPS program in case it is currently running.
	MC_DISABLE( P_MC1, MC1_ERPS1 );
	// Set starting logical address to write RPS commands.
	pRPS = c->RPSBuf.LogicalBase;

	// Construct RPS program in RPSBuf DMA buffer ----------------------------------------
	
	// Wait for Start trigger from ReadADC().
	WRITE_RPS( pRPS, RPS_PAUSE | RPS_SIGADC );

	// SAA7146 BUG WORKAROUND
	// Do a dummy DEBI Write.  This is necessary because the first RPS DEBI Write following a
	// non-RPS DEBI write seems to always fail.  If we don't do this dummy write, the ADC gain
	// might not be set to the value required for the first slot in the poll list; the ADC
	// gain would instead remain unchanged from the previously programmed value.
	WRITE_RPS( pRPS, RPS_LDREG | (P_DEBICMD >> 2) );		// Write DEBI Write command and address to shadow RAM.
	WRITE_RPS( pRPS, DEBI_CMD_WRWORD | LP_GSEL );
	WRITE_RPS( pRPS, RPS_LDREG | (P_DEBIAD >> 2) );			// Write DEBI immediate data to shadow RAM:
	WRITE_RPS( pRPS, GSEL_BIPOLAR5V );						//   arbitrary immediate data value.
	WRITE_RPS( pRPS, RPS_CLRSIGNAL | RPS_DEBI );			// Reset "shadow RAM uploaded" flag.
	WRITE_RPS( pRPS, RPS_UPLOAD    | RPS_DEBI );			// Invoke shadow RAM upload.
	WRITE_RPS( pRPS, RPS_PAUSE     | RPS_DEBI );			// Wait for shadow upload to finish.

	// Digitize all slots in the poll list.  This is implemented as a for loop to limit the slot
	// count to 16 in case the application forgot to set the EOPL flag in the final slot.
	for ( c->AdcItems = 0; c->AdcItems < 16; c->AdcItems++ )
	{
		// Convert application's poll list item to private board class format.  Each app poll list item
		// is an U8 with form (EOPL,x,x,RANGE,CHAN<3:0>), where RANGE code indicates 0 = +-10V, 1 = +-5V,
		// and EOPL = End of Poll List marker.
		LocalPPL = ( *ppl << 8 ) | ( *ppl & 0x10 ? GSEL_BIPOLAR5V : GSEL_BIPOLAR10V );

		// Switch ADC analog gain.
		WRITE_RPS( pRPS, RPS_LDREG | (P_DEBICMD >> 2) );	// Write DEBI command and address to shadow RAM.
		WRITE_RPS( pRPS, DEBI_CMD_WRWORD | LP_GSEL );
		WRITE_RPS( pRPS, RPS_LDREG | (P_DEBIAD >> 2) );		// Write DEBI immediate data to shadow RAM:
		WRITE_RPS( pRPS, LocalPPL );
		WRITE_RPS( pRPS, RPS_CLRSIGNAL | RPS_DEBI );		// Reset "shadow RAM uploaded" flag.
		WRITE_RPS( pRPS, RPS_UPLOAD    | RPS_DEBI );		// Invoke shadow RAM upload.
		WRITE_RPS( pRPS, RPS_PAUSE     | RPS_DEBI );		// Wait for shadow upload to finish.

		// Select ADC analog input channel.
		WRITE_RPS( pRPS, RPS_LDREG | (P_DEBICMD >> 2) );	// Write DEBI command and address to shadow.
		WRITE_RPS( pRPS, DEBI_CMD_WRWORD | LP_ISEL );
		WRITE_RPS( pRPS, RPS_LDREG | (P_DEBIAD >> 2) );		// Write DEBI immediate data to shadow.
		WRITE_RPS( pRPS, LocalPPL );
		WRITE_RPS( pRPS, RPS_CLRSIGNAL | RPS_DEBI );		// Reset "shadow RAM uploaded" flag.
		WRITE_RPS( pRPS, RPS_UPLOAD    | RPS_DEBI );		// Invoke shadow RAM upload.
		WRITE_RPS( pRPS, RPS_PAUSE     | RPS_DEBI );		// Wait for shadow upload to finish.

		// Delay at least 10 microseconds for analog input settling.  Instead of padding with NOPs,
		// we use RPS_JUMP instructions here; this allows us to produce a longer delay than is
		// possible with NOPs because each RPS_JUMP flushes the RPS' instruction prefetch pipeline.

		// 32bit DMA device.   
		PhysicalBase = S626MOD_cpu_to_le32( c->RPSBuf.PhysicalBase);
		JmpAdrs = PhysicalBase + (U32) ((unsigned char *) pRPS - (unsigned char *) c->RPSBuf.LogicalBase);

		for ( i = 0; i < ( 10 * RPSCLK_PER_US / 2); i++ )
		{
			JmpAdrs += 8;							// Repeat to implement time delay:
			WRITE_RPS( pRPS, RPS_JUMP );					//   Jump to next RPS instruction.
			WRITE_RPS( pRPS, JmpAdrs );
		}
		
		// Start ADC by pulsing GPIO1.
		WRITE_RPS( pRPS, RPS_LDREG | (P_GPIO >> 2) );		// Begin ADC Start pulse.
		WRITE_RPS( pRPS, GPIO_BASE | GPIO1_LO );
		WRITE_RPS( pRPS, RPS_NOP );							// VERSION 2.03 CHANGE: STRETCH OUT ADC START PULSE.
		WRITE_RPS( pRPS, RPS_LDREG | (P_GPIO >> 2) );		// End ADC Start pulse.
		WRITE_RPS( pRPS, GPIO_BASE | GPIO1_HI );

		// Wait for ADC to complete (GPIO2 is asserted high when ADC not busy) and for data
		// from previous conversion to shift into FB BUFFER 1 register.
		WRITE_RPS( pRPS, RPS_PAUSE | RPS_GPIO2 );			// Wait for ADC done.

		// Transfer ADC data from FB BUFFER 1 register to MyADCdata[] DMA buffer.
		WRITE_RPS( pRPS, RPS_STREG | ( BUGFIX_STREG( P_FB_BUFFER1 ) >> 2 ) );
		PhysicalBase = S626MOD_cpu_to_le32( c->ANABuf.PhysicalBase);
		WRITE_RPS( pRPS, PhysicalBase + ( c->AdcItems << 2 ) );

		// If this slot's EndOfPollList flag is set, all channels have now been processed.
		if ( *ppl++ & EOPL )
		{
			c->AdcItems++;		// Adjust poll list item count.
			break;				// Exit poll list processing loop.
		}
	}

	// VERSION 2.01 CHANGE: DELAY CHANGED FROM 250NS to 2US.
	// Allow the ADC to stabilize for 2 microseconds before starting the final (dummy) conversion.
	// This delay is necessary to allow sufficient time between last conversion finished and the start
	// of the dummy conversion.  Without this delay, the last conversion's data value is sometimes set
	// to the previous conversion's data value.
	for ( n = 0; n < ( 2 * RPSCLK_PER_US ); n++ ) {
		WRITE_RPS( pRPS, RPS_NOP );
	}

	// Start a dummy conversion to cause the data from the last conversion of interest to be shifted in.
	WRITE_RPS( pRPS, RPS_LDREG | (P_GPIO >> 2) );				// Begin ADC Start pulse.
	WRITE_RPS( pRPS, GPIO_BASE | GPIO1_LO );
	WRITE_RPS( pRPS, RPS_NOP );								// VERSION 2.03 CHANGE: STRETCH OUT ADC START PULSE.
	WRITE_RPS( pRPS, RPS_LDREG | (P_GPIO >> 2) );				// End ADC Start pulse.
	WRITE_RPS( pRPS, GPIO_BASE | GPIO1_HI );

	// Wait for the data from the last conversion of interest to arrive in FB BUFFER 1 register.
	WRITE_RPS( pRPS, RPS_PAUSE | RPS_GPIO2 );					// Wait for ADC done.
		
	// Transfer final ADC data from FB BUFFER 1 register to MyADCdata[] DMA buffer.
	WRITE_RPS( pRPS, RPS_STREG | ( BUGFIX_STREG( P_FB_BUFFER1 ) >> 2 ) );
	PhysicalBase = S626MOD_cpu_to_le32( c->ANABuf.PhysicalBase);
 	WRITE_RPS( pRPS, PhysicalBase + ( c->AdcItems << 2 ) );

	// Indicate ADC scan loop is finished.
	WRITE_RPS( pRPS, RPS_CLRSIGNAL | RPS_SIGADC );					// Signal ReadADC() that scan is done.

	// Restart RPS program at its beginning.
	WRITE_RPS( pRPS, RPS_JUMP );	
	// Branch to start of RPS program.
	PhysicalBase = S626MOD_cpu_to_le32( c->RPSBuf.PhysicalBase );
	WRITE_RPS( pRPS, PhysicalBase);
	// End of RPS program build  ------------------------------------------------------------
	// Start executing the RPS program.
	MC_ENABLE( P_MC1, MC1_ERPS1 );

}

void S626CORE_ResetADC( HBD bd, U8 *ppl )
{
	struct CORE_OBJ	*c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Load poll list if board is open.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			ResetADC( c, ppl );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Execute one ADC.  AdcSpec has form (x,x,x,RANGE,CHAN<3:0>), where RANGE code indicates 0 = +-10V, 1 = +-5V.

U16 S626CORE_SingleADC( HBD bd, U16 AdcSpec )
{
	struct CORE_OBJ	*c;
	U32				AdcData = 0;
	U32				GpioImage;
	int				i;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// If board is open ...
		if ( c->IsOpen )
		{
			// Convert application's ADC specification into form appropriate for register programming.
			AdcSpec = ( AdcSpec << 8 ) | ( AdcSpec & 0x10 ? GSEL_BIPOLAR5V : GSEL_BIPOLAR10V );

			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			// Switch ADC analog gain.
			DEBIwrite( c, LP_GSEL, AdcSpec );				// Set gain.

			// Select ADC analog input channel.
			DEBIwrite( c, LP_ISEL, AdcSpec );				// Select channel.

			// Delay 10 microseconds for analog input settling.
			for ( i = 0; i < ( 10 * 33 ); i++ )
				RR7146( P_PSR );

			// Start ADC by pulsing GPIO1 low.
			GpioImage = RR7146( P_GPIO );
			WR7146( P_GPIO, GpioImage & ~GPIO1_HI );			// Assert ADC Start command
			WR7146( P_GPIO, GpioImage & ~GPIO1_HI );			//   and stretch it out.
			WR7146( P_GPIO, GpioImage & ~GPIO1_HI );
			WR7146( P_GPIO, GpioImage | GPIO1_HI );				// Negate ADC Start command.

			// Wait for ADC to complete (GPIO2 is asserted high when ADC not busy) and for data
			// from previous conversion to shift into FB BUFFER 1 register.
			while ( !( RR7146( P_PSR ) & PSR_GPIO2 ) );			// Wait for ADC done.

			// Allow the ADC to stabilize for 4 microseconds before starting the final (dummy) conversion.
			// This delay is necessary to allow sufficient time between last conversion finished and the start
			// of the dummy conversion.  Without this delay, the last conversion's data value is sometimes set
			// to the previous conversion's data value.
			for ( i = 0; i < ( 4 * 33 ); i++ )
				RR7146( P_PSR );

			// Start a dummy conversion to cause the data from the previous conversion to be shifted in.
			GpioImage = RR7146( P_GPIO );
			WR7146( P_GPIO, GpioImage & ~GPIO1_HI );			// Assert ADC Start command
			WR7146( P_GPIO, GpioImage & ~GPIO1_HI );			//   and stretch it out.
			WR7146( P_GPIO, GpioImage & ~GPIO1_HI );
			WR7146( P_GPIO, GpioImage | GPIO1_HI );				// Negate ADC Start command.

			// Wait for the data to arrive in FB BUFFER 1 register.
			while ( !( RR7146( P_PSR ) & PSR_GPIO2 ) );			// Wait for ADC done.

			// Fetch ADC data from audio interface's input shift register.
			AdcData = RR7146( P_FB_BUFFER1 );					// Fetch ADC data.

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}

	// Return the digitized value.
	return (U16)( AdcData >> 16 );
}

/////////////////////////////////////////////////////////////////////////////
// Initialize ADC.

static void InitADC( struct CORE_OBJ *c )
{
	U8	PollList;
	U16	AdcData;
	U16	StartVal;
	U16	i;
	U32     PhysicalBase;
	// Set up TSL1 slot list, which is used to control the accumulation of ADC data:
	//  RSD1   = shift data in on SD1.
	//  SIB_A1 = store data U8 at next available location in FB BUFFER1 register.
	WR7146( P_TSL1    , RSD1 | SIB_A1 );							// Fetch ADC high data U8.
	WR7146( P_TSL1 + 4, RSD1 | SIB_A1 | EOS );						// Fetch ADC low data U8; end of TSL1.

	// enab TSL1 slot list so that it executes all the time.
	WR7146( P_ACON1, ACON1_ADCSTART );

	// Initialize RPS registers used for ADC. 
	assert( c->RPSBuf.PhysicalBase <= (PVOID) 0xffffffff);

	PhysicalBase = S626MOD_cpu_to_le32( c->RPSBuf.PhysicalBase );	// Physical start of RPS program.
	WR7146( P_RPSADDR1, PhysicalBase );	// Physical start of RPS program.
	WR7146( P_RPSPAGE1, 0 );										// RPS program performs no explicit mem writes.
	WR7146( P_RPS1_TOUT, 0 );										// Disable RPS timeouts.

	// SAA7146 BUG WORKAROUND.
	// Initialize SAA7146 ADC interface to a known state by invoking ADCs until FB BUFFER 1 register
	// shows that it is correctly receiving ADC data.  This is necessary because the SAA7146 ADC
	// interface does not start up in a defined state after a PCI reset.

	PollList = EOPL;												// Create a simple polling list for analog input channel 0.
	ResetADC( c, &PollList );
	ReadADC( c, &AdcData );		// Get initial ADC value.
	StartVal = AdcData;

	// VERSION 2.01 CHANGE: TIMEOUT ADDED TO PREVENT HANGED EXECUTION.
	// Invoke ADCs until the new ADC value differs from the initial value or a timeout occurs.  The
	// timeout protects against the possibility that the driver is restarting and the ADC data is a
	// fixed value resulting from the applied ADC analog input being unusually quiet or at the rail.
	for ( i = 0; i < 500; i++ )
	{
		ReadADC( c, &AdcData );
		if ( AdcData != StartVal )
			break;
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////  DAC FUNCTIONS  ////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////


// Slot 0 base settings.
#define VECT0			( XSD2 | RSD3 | SIB_A2 )		// Slot 0 always shifts in 0xFF and stores it to FB_BUFFER2.

// TrimDac LogicalChan-to-PhysicalChan mapping table.
static U8 trimchan[] = { 10, 9, 8, 3, 2, 7, 6, 1, 0, 5, 4 };

// TrimDac LogicalChan-to-EepromAdrs mapping table.
static U8 trimadrs[] = { 0x40, 0x41, 0x42, 0x50, 0x51, 0x52, 0x53, 0x60, 0x61, 0x62, 0x63 };

////////////////////////////////////////////////////////////////////
// Initialize DAC interface.

static void InitDAC( struct CORE_OBJ *c )
{
	register U32 *pPhysBuf;
	U32 physval;
	// Init Audio2's output DMAC attributes: burst length = 1 DWORD, threshold = 1 DWORD.
	WR7146( P_PCI_BT_A, 0 );

	// Init Audio2's output DMA physical addresses.  The protection address is set to 1 DWORD past the base address
	// so that a single DWORD will be transferred each time a DMA transfer is enabled.

	pPhysBuf = (U32 *)c->ANABuf.PhysicalBase + DAC_WDMABUF_OS;
	//printf("anabuf %p, pphysbuf %p \n", c->ANABuf.PhysicalBase, pPhysBuf);

	physval = S626MOD_cpu_to_le32( pPhysBuf);
	WR7146( P_BASEA2_OUT, physval );				// Buffer base adrs.
	physval = S626MOD_cpu_to_le32( pPhysBuf + 1);
	WR7146( P_PROTA2_OUT, physval); 		// Protection address.

	// Cache Audio2's output DMA buffer logical address.  This is where DAC data is buffered for A2 output DMA transfers.
	c->pDacWBuf = (U32 *)c->ANABuf.LogicalBase + DAC_WDMABUF_OS;

	// Audio2's output channels does not use paging.  The protection violation handling bit is set so that the
	// DMAC will automatically halt and its PCI address pointer will be reset when the protection address is reached.
	WR7146( P_PAGEA2_OUT, 8 );

	// Initialize time slot list 2 (TSL2), which is used to control the clock generation for and serialization of data
	// to be sent to the DAC devices.  Slot 0 is a NOP that is used to trap TSL execution; this permits other slots
	// to be safely modified without first turning off the TSL sequencer (which is apparently impossible to do).  Also,
	// SD3 (which is driven by a pull-up resistor) is shifted in and stored to the MSB of FB_BUFFER2 to be used as
	// evidence that the slot sequence has not yet finished executing.
	SETVECT( 0, XSD2 | RSD3 | SIB_A2 | EOS );				// Slot 0: Trap TSL execution, shift 0xFF into FB_BUFFER2.

	// Initialize slot 1, which is constant.  Slot 1 causes a DWORD to be transferred from audio channel 2's output
	// FIFO to the FIFO's output buffer so that it can be serialized and sent to the DAC during subsequent slots.  All
	// remaining slots are dynamically populated as required by the target DAC device.
	SETVECT( 1, LF_A2 );	// Slot 1: Fetch DWORD from Audio2's output FIFO.

	// Start DAC's audio interface (TSL2) running.
	WR7146( P_ACON1, ACON1_DACSTART );
}

//////////////////////////////////////////////////////////////////////////////
// Write/return setpoint to/from a TrimDAC channel.

static void WriteTrimDAC( struct CORE_OBJ *c, U8 LogicalChan, U8 DacData )
{
	U32 chan;

	// Abort if illegal TrimDac channel number specified.
	ValidateParm( LogicalChan, NUM_TRIMDACS );

	// Start critical segment.
	//S626MOD_CriticalBegin( c->Board );	// done at upper level

	// Save the new setpoint in case the application needs to read it back later.
	c->TrimSetpoint[LogicalChan] = (U8)DacData;

	// Map logical channel number to physical channel number.
	chan = (U32)trimchan[LogicalChan];

	// Set up TSL2 records for TrimDac write operation.  All slots shift 0xFF in from pulled-up SD3 so that the end
	// of the slot sequence can be detected.
	SETVECT( 2, XSD2 | XFIFO_1 | WS3 );				// Slot 2: Send high U8 to target TrimDac.
	SETVECT( 3, XSD2 | XFIFO_0 | WS3 );				// Slot 3: Send low U8 to target TrimDac.
	SETVECT( 4, XSD2 | XFIFO_3 | WS1 );				// Slot 4: Send NOP high U8 to DAC0 to keep clock running.
	SETVECT( 5, XSD2 | XFIFO_2 | WS1 | EOS );		// Slot 5: Send NOP low U8 to DAC0.

	// Construct and transmit target DAC's serial packet: ( 0000 AAAA ),( DDDD DDDD ),( 0x00 ),( 0x00 ) where A<3:0> is the
	// DAC channel's address, and D<7:0> is the DAC setpoint.  Append a WORD value (that writes a channel 0 NOP command to a
	// non-existent main DAC channel) that serves to keep the clock running after the packet has been sent to the target DAC.
	SendDAC( c, ( (U32)chan << 8 )	// Address the DAC channel within the trimdac device.
		| (U32)DacData );					// Include DAC setpoint data.

	// End critical segment.
	//S626MOD_CriticalEnd( c->Board );		// done at upper level
}

void S626CORE_WriteTrimDAC( HBD bd, U8 logchan, U8 val )
{
	struct CORE_OBJ	*c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Write to trimdac if board is open.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			WriteTrimDAC( c, logchan, val );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}

U8 S626CORE_ReadTrimDAC( HBD bd, U8 logchan )
{
	struct CORE_OBJ *c;
	U8 rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Validate channel number.
		ValidateParmRtnval( logchan, NUM_TRIMDACS );

		// Return the setpoint image.
		if ( c->IsOpen )
			rtnval = c->TrimSetpoint[logchan];
	}

	return rtnval;
}

////////////////////////////////////////////////////////////////////
// Read all TrimDac setpoints from EEPROM and write to TrimDacs.
// Logical TrimDac channels are ordered as follows:
// ADCZero, ADCGain10V, ADCGain5V, DACZero[4], DACGain[4].

static void LoadTrimDACs( struct CORE_OBJ *c )
{
	register U8 i;

	// Copy TrimDac setpoint values from EEPROM to TrimDacs.
	for ( i = 0; i < sizeof(trimchan); i++ )
		WriteTrimDAC( c, i, I2Cread( c, trimadrs[i] ) );
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Write setpoint to a DAC channel.  If polarity of new setpoint differs from that of the
// current setpoint, two DAC writes are performed: the first write switches the DAC output to
// 0V and the second write switches the output to the desired setpoint.  This prevents a glitch
// from occuring on the DAC output, which could be caused by the polarity change occuring before
// the new DAC setpoint value is programmed.

void S626CORE_WriteDAC( HBD bd, U16 chan, S16 dacdata )
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if invalid channel number specified.
		ValidateParm( chan, NUM_DACS );

		// If board is open ...
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			// If new setpoint has polarity that differs from current setpoint, write an intermediate
			// output setpoint near 0V to prevent the DAC output from glitching.
			if ( ( ( dacdata ^ c->DacSetpoint[chan] ) & 0x8000 ) != 0 )
				SetDAC( c, chan, (S16)( ( dacdata < 0 ) ? 0 : -1 ) );

			// Write new DAC setpoint.
			SetDAC( c, chan, dacdata );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////
// Return DAC channel's cached setpoint value.

S16 S626CORE_ReadDAC( HBD bd, U16 chan )
{
	struct CORE_OBJ *c;

	// Abort if illegal DAC channel number specified.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	return ( c->IsOpen ) ? c->DacSetpoint[chan] : 0;
}

/////////////////////////////////////////////////////////////////////////////////
// Private helper function: Write setpoint to an application DAC channel.

static void SetDAC( struct CORE_OBJ *c, U16 chan, S16 dacdata )
{
	register U16	signmask;
	register U32	WSImage;

	// Cache new setpoint.
	c->DacSetpoint[chan] = dacdata;

	// Adjust DAC data polarity and set up Polarity Control Register image.
	signmask = 1 << chan;
	if ( dacdata < 0 )
	{
		dacdata = -dacdata;
		c->Dacpol |= signmask;
	}
	else
		c->Dacpol &= ~signmask;

	// Limit DAC setpoint value to valid range.
	if ( (U16)dacdata > 0x1FFF )
		dacdata = 0x1FFF;

	// Set up TSL2 records (aka "vectors") for DAC update.  Vectors V2 and V3 transmit the setpoint to the target DAC.
	// V4 and V5 send data to a non-existent TrimDac channel just to keep the clock running after sending data to the target
	// DAC.  This is necessary to eliminate the clock glitch that would otherwise occur at the end of the target DAC's serial
	// data stream.  When the sequence restarts at V0 (after executing V5), the gate array automatically disables gating for
	// the DAC clock and all DAC chip selects.
	WSImage = ( chan & 2 ) ? WS1 : WS2;					// Choose DAC chip select to be asserted.
	SETVECT( 2, XSD2 | XFIFO_1 | WSImage );				// Slot 2: Transmit high data byte to target DAC.
	SETVECT( 3, XSD2 | XFIFO_0 | WSImage );				// Slot 3: Transmit low data byte to target DAC.
	SETVECT( 4, XSD2 | XFIFO_3 | WS3 );					// Slot 4: Transmit to non-existent TrimDac channel to keep clock
	SETVECT( 5, XSD2 | XFIFO_2 | WS3 | EOS );			// Slot 5:   running after writing target DAC's low data byte.

	// Construct and transmit target DAC's serial packet: ( A10D DDDD ),( DDDD DDDD ),( 0x0F ),( 0x00 ) where A is chan<0>, and D<12:0>
	// is the DAC setpoint.  Append a WORD value (that writes to a non-existent TrimDac channel) that serves to keep the clock running
	// after the packet has been sent to the target DAC.
	SendDAC( c, 0x0F000000							// Continue clock after target DAC data (write to non-existent trimdac).
		| 0x00004000									// Address the two main dual-DAC devices (TSL's chip select enables target device).
		| ( (U32)( chan & 1 ) << 15 )			// Address the DAC channel within the device.
		| (U32)dacdata );						// Include DAC setpoint data.
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Private helper function: Transmit serial data to DAC via Audio channel 2.
// Assumes: (1) TSL2 slot records initialized, and (2) Dacpol contains valid target image.

static void SendDAC( struct CORE_OBJ *c, U32 val )
{
	// START THE SERIAL CLOCK RUNNING -------------

	// Assert DAC polarity control and enable gating of DAC serial clock and audio bit stream signals.  At
	// this point in time we must be assured of being in time slot 0.  If we are not in slot 0, the serial
	// clock and audio stream signals will be disabled; this is because the following DEBIwrite statement
	// (which enables signals to be passed through the gate array) would execute before the trailing edge of
	// WS1/WS3 (which turns off the signals), thus causing the signals to be inactive during the DAC write.
	DEBIwrite( c, LP_DACPOL, c->Dacpol );

	// TRANSFER OUTPUT DWORD VALUE INTO A2'S OUTPUT FIFO ----------------

	// Copy DAC setpoint value to DAC's output DMA buffer.
	S626MOD_WriteMem( c->pDacWBuf, val );	//*c->pDacWBuf = val;

	// enab the output DMA transfer.  This will cause the DMAC to copy the DAC's data value to A2's output
	// FIFO.  The DMA transfer will then immediately terminate because the protection address is reached
	// upon transfer of the first DWORD value.
	MC_ENABLE( P_MC1, MC1_A2OUT );

	// While the DMA transfer is executing ...

	// Reset Audio2 output FIFO's underflow flag (along with any other FIFO underflow/overflow flags).  When set,
	// this flag will indicate that we have emerged from slot 0.
	WR7146( P_ISR, ISR_AFOU );

	// Wait for the DMA transfer to finish so that there will be data available in the FIFO when time slot 1
	// tries to transfer a DWORD from the FIFO to the output buffer register.  We test for DMA Done by polling the
	// DMAC enable flag; this flag is automatically cleared when the transfer has finished.
	while ( ( RR7146( P_MC1 ) & MC1_A2OUT ) != 0 );

	// START THE OUTPUT STREAM TO THE TARGET DAC --------------------

	// FIFO data is now available, so we enable execution of time slots 1 and higher by clearing the
	// EOS flag in slot 0.  Note that SD3 will be shifted in and stored in FB_BUFFER2 for end-of-slot-list detection.
	SETVECT( 0, XSD2 | RSD3 | SIB_A2 );

	// Wait for slot 1 to execute to ensure that the Packet will be transmitted.  This is detected
	// by polling the Audio2 output FIFO underflow flag, which will be set when slot 1 execution has
	// finished transferring the DAC's data DWORD from the output FIFO to the output buffer register.
	while ( ( RR7146( P_SSR ) & SSR_AF2_OUT ) == 0 );

	// Set up to trap execution at slot 0 when the TSL sequencer cycles back to slot 0 after executing the
	// EOS in slot 5.  Also, simultaneously shift out and in the 0x00 that is ALWAYS the value stored in the
	// last byte to be shifted out of the FIFO's DWORD buffer register.
	SETVECT( 0, XSD2 | XFIFO_2 | RSD2 | SIB_A2 | EOS );

	// WAIT FOR THE TRANSACTION TO FINISH -----------------------

	// Wait for the TSL to finish executing all time slots before exiting this function.  We must do this so that
	// the next DAC write doesn't start, thereby enabling clock/chip select signals:
	//   1.	Before the TSL sequence cycles back to slot 0, which disables the clock/cs signal gating and traps slot
	//		list execution.  If we have not yet finished slot 5 then the clock/cs signals are still gated and we have
	//		not finished transmitting the stream.
	//   2.	While slots 2-5 are executing due to a late slot 0 trap.  In this case, the slot sequence is currently
	//		repeating, but with clock/cs signals disabled.  We must wait for slot 0 to trap execution before setting
	//		up the next DAC setpoint DMA transfer and enabling the clock/cs signals.
	// To detect the end of slot 5, we test for the FB_BUFFER2 MSB contents to be equal to 0xFF.  If the TSL has
	// not yet finished executing slot 5 ...
	if ( ( RR7146( P_FB_BUFFER2 ) & 0xFF000000 ) != 0 )
	{
		// The trap was set on time and we are still executing somewhere in slots 2-5, so we now wait for slot 0 to execute
		// and trap TSL execution.  This is detected when FB_BUFFER2 MSB changes from 0xFF to 0x00, which slot 0 causes to
		// happen by shifting out/in on SD2 the 0x00 that is always referenced by slot 5.
		while ( ( RR7146( P_FB_BUFFER2 ) & 0xFF000000 ) != 0 );
	}

	// Either (1) we were too late setting the slot 0 trap; the TSL sequencer restarted slot 0 before we could set the EOS
	// trap flag, or (2) we were not late and execution is now trapped at slot 0.  In either case, we must now change slot 0
	// so that it will store value 0xFF (instead of 0x00) to FB_BUFFER2 next time it executes.  In order to do this, we
	// reprogram slot 0 so that it will shift in SD3, which is driven only by a pull-up resistor.
	SETVECT( 0, RSD3 | SIB_A2 | EOS );

	// Wait for slot 0 to execute, at which time the TSL is setup for the next DAC write.  This is detected when FB_BUFFER2
	// MSB changes from 0x00 to 0xFF.
	while ( ( RR7146( P_FB_BUFFER2 ) & 0xFF000000 ) == 0 );
}

////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////  PRIVATE HIGH-LEVEL FUNCTION HELPERS  ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
// Define new image for MISC2 register and write it to MISC2 reg.

static void WriteMISC2( struct CORE_OBJ *c, U16 NewImage )
{
	DEBIwrite( c, LP_MISC1, MISC1_WENABLE );	// enab writes to MISC2 register.
	DEBIwrite( c, LP_WRMISC2, NewImage );		// Write new image to MISC2.
	DEBIwrite( c, LP_MISC1, MISC1_WDISABLE );	// Disable writes to MISC2.
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Return image of what was last written to the MISC2 register.

#define BIT14	0x4000
#define BIT13	0x2000

#define RBMASK	( MISC2_BATT_ENABLE | MISC2_WDENABLE | MISC2_WDPERIOD_MASK )

static U16 WRMISC2image( struct CORE_OBJ *c )
{
	register U16	c3;
	register U16	c2;
	register U16	c1;
	U16				retval;

	// Start critical segment.
	//S626MOD_CriticalBegin( c->Board );	// done in upper level.

	// Fetch DIO 0-5 mode flags.  These must be read from the counter B control registers as
	// they are write-only bits in the MISC2 register and therefore have no direct readback from MISC2.
	c3 = DEBIread( c, LP_CR2B );		// DIO 0-1 modes.
	c2 = DEBIread( c, LP_CR1B );		// DIO 2-3 modes.
	c1 = DEBIread( c, LP_CR0B );		// DIO 4-5 modes.

	// Fetch BackupEnable, WatchdogEnable and WatchdogInterval flags, and combine with
	// ChargeEnable (image only, no readback register is available).
	retval = ( DEBIread( c, LP_RDMISC2 ) & RBMASK ) | c->ChargeEnabled;

	// End critical segment.
	//S626MOD_CriticalEnd( c->Board );		// done in upper level.

	// Assemble all the bits to produce an image of the current state of the WRMISC2 register.
	return retval
		| ( ( c3 & BIT13 ) >>  4 )		// DIO 5 mode.
		| ( ( c3 & BIT14 ) >>  6 )		// DIO 4 mode.
		| ( ( c2 & BIT13 ) >>  6 )		// DIO 3 mode.
		| ( ( c2 & BIT14 ) >>  8 )		// DIO 2 mode.
		| ( ( c1 & BIT13 ) >>  8 )		// DIO 1 mode.
		| ( ( c1 & BIT14 ) >> 10 );		// DIO 0 mode.
}

////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////  BATTERY CHARGING AND BACKUP FUNCTIONS  //////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////
// Return non-zero if battery backup is enabled.

U16 S626CORE_BackupEnableGet( HBD bd )
{
	struct CORE_OBJ	*c;
	U16				retval;

	// Abort if illegal parameter specified.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Execute BackupEnableGet if board is open.
	if ( c->IsOpen )
	{
		// Start critical segment.
		S626MOD_CriticalBegin( c->Board );

		retval = DEBIread( c, LP_RDMISC2 ) & MISC2_BATT_ENABLE;

		// End critical segment.
		S626MOD_CriticalEnd( c->Board );
	} else {
		retval = 0;
	}

	return retval;

}

////////////////////////////////////////////////////////////////////
// enab/disable battery backup.  Enabled if enab is non-zero.

void S626CORE_BackupEnableSet( HBD bd, U16 enab )
{
	struct CORE_OBJ	*c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// If board is open ...
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			// Construct new image of MISC2 register and write it to MISC2.
			WriteMISC2( c, (U16)( enab ? ( WRMISC2image( c ) | MISC2_BATT_ENABLE ) : ( WRMISC2image( c ) & ~MISC2_BATT_ENABLE ) ) );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}


/////////////////////////////////////////////////////
// Return non-zero if battery charging is enabled.

U16 S626CORE_ChargeEnableGet( HBD bd )
{
	struct CORE_OBJ	*c;

	// Abort if illegal parameter specified.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	return ( c->IsOpen ) ? c->ChargeEnabled : 0;
}

//////////////////////////////////////////////////////////////////////
// enab/disable battery charging.  Enabled if enab is non-zero.

void S626CORE_ChargeEnableSet( HBD bd, U16 enab )
{
	struct CORE_OBJ	*c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// If board is open ...
		if ( c->IsOpen )
		{
			// Init image of WRMISC2 Battery Charge Enabled control bit.  This image is used when the
			// state of the control bit is queried as it has no direct hardware readback mechanism.
			c->ChargeEnabled = enab ? WRMISC2_CHARGE_ENABLE : 0;

			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			// Program the enable output for the battery charger.
			WriteMISC2( c, (U16)( ( WRMISC2image( c ) & ~WRMISC2_CHARGE_ENABLE ) | c->ChargeEnabled ) );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}


////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////  WATCHDOG FUNCTIONS  /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////
// Return non-zero if the watchdog has timed out.

U16 S626CORE_WatchdogTimeout( HBD bd )
{
	struct CORE_OBJ	*c;
	U16				retval;

	// Abort if illegal parameter specified.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Indicate whether watchdog timed out.
	if ( c->IsOpen )
	{
		// Start critical segment.
		S626MOD_CriticalBegin( c->Board );

		retval = DEBIread( c, LP_MISC1 ) & RDMISC1_WDTIMEOUT;

		// End critical segment.
		S626MOD_CriticalEnd( c->Board );
	} else {
		retval = 0;
	}

	return retval;

}

//////////////////////////////////////////////////////
// Return non-zero if the watchdog is enabled.

U16 S626CORE_WatchdogEnableGet( HBD bd )
{
	struct CORE_OBJ	*c;
	U16				retval;

	// Abort if illegal parameter specified.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Indicate whether watchdog is enabled.
	if ( c->IsOpen )
	{
		// Start critical segment.
		S626MOD_CriticalBegin( c->Board );

		retval = DEBIread( c, LP_RDMISC2 ) & MISC2_WDENABLE;

		// End critical segment.
		S626MOD_CriticalEnd( c->Board );
	} else {
		retval = 0;
	}

	return retval;

}

/////////////////////////////////////////////////////////////////////////
// enab/disable the watchdog timer.  Enabled if enab is non-zero.

void S626CORE_WatchdogEnableSet( HBD bd, U16 enab )
{
	struct CORE_OBJ	*c;
	register U16	image;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// If board is open ...
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			// Get image of value last written to MISC2.
			image = WRMISC2image( c );

			// If watchdog is to be enabled ...
			if ( enab )
			{
				// Construct new image to be written to MISC2.
				image = MISC2_WDENABLE					// enab the WD timer.
					| ( image & ~MISC2_WDPERIOD_MASK )	// Get image of value last written to MISC2.
					| c->WDInterval;					// Combine with image of WD interval control.

				// The MISC2 register must be written twice because the WD interval control bits are forced
				// to zero any time the WD enable is inactive.  The first write sets WD enable active and
				// releases the forced reset on the WD interval control bits.  The second write programs the
				// WD interval bits since they are no longer forced to zero.
				DEBIwrite( c, LP_MISC1, MISC1_WENABLE );	// enab write access to WD enable/period control in MISC2.
				DEBIwrite( c, LP_WRMISC2, image );			// enab WD timer and enable WD period control programming.
				DEBIwrite( c, LP_WRMISC2, image );			// Program WD period.
				DEBIwrite( c, LP_MISC1, MISC1_WDISABLE );	// Disable write access to WD enable/period control.
			}
			else
			{
				// Disable the WD timer.  This also causes the gate array to automatically reset the programmed
				// WD interval control bits to zero (and to hold them there until the WD timer is again enabled).
				WriteMISC2( c, (U16)( image & ~MISC2_WDENABLE ) );
			}

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}

/////////////////////////////////////////////////////////////////////
// Return/set the programmed watchdog timer interval.
// Legal timer interval values: 0, 1, 2 or 3.

U16 S626CORE_WatchdogPeriodGet( HBD bd )	// Return WD interval.
{
	struct CORE_OBJ	*c;
	U16				m2;
	U16				rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// If board is open ...
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			// Fetch WD enable and interval control bits.
			m2 = DEBIread( c, LP_RDMISC2 );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );

			// If WD is enabled, simply return the programmed interval control bits.  If disabled, we
			// instead return an image of the desired interval control bits since the physical control bits
			// are forced to zero by the gate array whenever the WD timer is disabled.
			rtnval = ( m2 & MISC2_WDENABLE ) ? ( m2 & MISC2_WDPERIOD_MASK ) : c->WDInterval;
		}
	}

	return rtnval;
}

void S626CORE_WatchdogPeriodSet( HBD bd, U16 value )	// Set WD interval.
{
	struct CORE_OBJ	*c;
	register U16	image;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if parameter is not valid.
		ValidateParm( value, MISC2_WDPERIOD_MASK );

		// If board is open ...
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			// Cache the new WD interval.
			c->WDInterval = value;

			// Get the value last written to the MISC2 register.
			image = WRMISC2image( c );

			// If the WD timer is enabled, program the new interval value.  We skip this if the WD timer
			// is disabled because the gate array will be forcing the WD interval control bits to zero;
			// this doesn't matter, because we have a copy of the interval control bits in WDInterval.
			if ( image & MISC2_WDENABLE )
				WriteMISC2( c, (U16)( ( image & ~MISC2_WDPERIOD_MASK ) | value ) );		// Program new interval.

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}

///////////////////////////////////////////////////////////////////
// Reset the watchdog timer.

void S626CORE_WatchdogReset( HBD bd )
{
	struct CORE_OBJ	*c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Reset watchdog timer.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			DEBIwrite( c, LP_WRMISC2, WRMISC2_WDCLEAR );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////  DIGITAL I/O FUNCTIONS  /////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
// All DIO functions address a group of DIO channels by means of "group" argument.
// group may be 0, 1 or 2, which correspond to DIO ports A, B and C, respectively.
/////////////////////////////////////////////////////////////////////////////////////////
// These arrays map a DIO group number into the various port addresses needed to access DIO registers.
static const U16 RDDIn[]		= { LP_RDDINA,    LP_RDDINB,    LP_RDDINC    };	// Read  physical input states.
static const U16 WRDOut[]	= { LP_WRDOUTA,   LP_WRDOUTB,   LP_WRDOUTC   };	// Write physical output states.
static const U16 RDEdgSel[]	= { LP_RDEDGSELA, LP_RDEDGSELB, LP_RDEDGSELC };	// Read  edge polarity.
static const U16 WREdgSel[]	= { LP_WREDGSELA, LP_WREDGSELB, LP_WREDGSELC };	// Write edge polarity.
static const U16 RDCapSel[]	= { LP_RDCAPSELA, LP_RDCAPSELB, LP_RDCAPSELC };	// Read  event capture enables.
static const U16 WRCapSel[]	= { LP_WRCAPSELA, LP_WRCAPSELB, LP_WRCAPSELC };	// Write event capture enables.
static const U16 RDCapFlg[]	= { LP_RDCAPFLGA, LP_RDCAPFLGB, LP_RDCAPFLGC };	// Read  event capture flags.
static const U16 RDIntSel[]	= { LP_RDINTSELA, LP_RDINTSELB, LP_RDINTSELC };	// Read  interrupt enables.
static const U16 WRIntSel[]	= { LP_WRINTSELA, LP_WRINTSELB, LP_WRINTSELC };	// Write interrupt enables.

/////////////////////////////////////////////////////////////////////////////////////////
// Initialize all DIO hardware to be consistent with states following a PCI hard reset.

static void DIOInit( struct CORE_OBJ *c )
{
	register U16 group;

	// Prepare to treat writes to WRCapSel as capture disables.
	DEBIwrite( c, LP_MISC1, MISC1_NOEDCAP );

	// For each group of sixteen channels ...
	for ( group = 0; group < 3; group++ )
	{
		DEBIwrite( c, WRIntSel[group], 0 );							// Disable all interrupts.
		DEBIwrite( c, WRCapSel[group], 0xFFFF );						// Disable all event captures.
		DEBIwrite( c, WREdgSel[group], 0 );							// Init all DIOs to default edge polarity.
		DEBIwrite( c, WRDOut[group], c->DOutImage[group] = 0 );	// Program all outputs to inactive state.
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
// Helper function: insert a bit value into a word.

static U16 DIOBitInsert( U16 WordVal, U16 chan, U16 BitVal )
{
	// Cache the channel's bit mask.
	register U16 Mask = 1 << ( chan & 15 );

	// Return modified word value.
	return BitVal ? ( WordVal | Mask ) : ( WordVal & ~Mask );
}

////////////////////////////////////////////////////////////////////////////
// Return the input states of a group of DIO channels.

U16 S626CORE_DIOGroupRead( HBD bd, U16 group )
{
	struct CORE_OBJ *c;
	U16				retval;

	// Abort if board number is not valid.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Abort if illegal DIO group number specified.
	ValidateParmRtnval( group, NUM_DIOBANKS );

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	// Return DIO input states.
	retval = DEBIread( c, RDDIn[group] );

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );

	return retval;
}

////////////////////////////////////////////////////////////////////////////
// Return the input state of a DIO channel.

U16 S626CORE_DIOChanRead( HBD bd, U16 chan )
{
	struct CORE_OBJ *c;
	U16				retval;

	// Abort if board number is not valid.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Abort if illegal DIO chan number specified.
	ValidateParmRtnval( chan, NUM_DIOCHANS );

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	// Return DIO input state.
	retval = ( DEBIread( c, RDDIn[chan >> 4] ) >> ( chan & 15 ) ) & 1;

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );

	return retval;
}

////////////////////////////////////////////////////////////////////////////
// Return/set physical output register states of a DIO channel group.

U16 S626CORE_DIOGroupWriteGet( HBD bd, U16 group )
{
	struct CORE_OBJ *c;

	// Abort if board number is not valid.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Abort if illegal DIO group number specified.
	ValidateParmRtnval( group, NUM_DIOBANKS );

	// Return DIO physical output states.
	return c->DOutImage[group];
}

void S626CORE_DIOGroupWriteSet( HBD bd, U16 group, U16 value )
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal DIO group number specified.
		ValidateParm( group, NUM_DIOBANKS );

		// Start critical segment.
		S626MOD_CriticalBegin( c->Board );

		// Modify DIO group output image and write to physical outputs.
		DEBIwrite( c, WRDOut[group], c->DOutImage[group] = value );

		// End critical segment.
		S626MOD_CriticalEnd( c->Board );
	}
}

////////////////////////////////////////////////////////////////////////////
// Return/set physical output register state of a DIO channel.

U16 S626CORE_DIOChanWriteGet( HBD bd, U16 chan )
{
	struct CORE_OBJ *c;

	// Abort if board number is not valid.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Abort if illegal DIO chan number specified.
	ValidateParmRtnval( chan, NUM_DIOCHANS );

	// Return DIO physical output state.
	return ( c->DOutImage[chan >> 4] >> ( chan & 15 ) ) & 1;
}

void S626CORE_DIOChanWriteSet( HBD bd, U16 chan, U16 value )
{
	register U16 group;
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal DIO chan number specified.
		ValidateParm( chan, NUM_DIOCHANS );

		// Cache the channel's group number.
		group = chan >> 4;

		// Start critical segment.
		S626MOD_CriticalBegin( c->Board );

		// Construct new DIO group image.
		c->DOutImage[group] = DIOBitInsert( c->DOutImage[group], chan, value );

		// Modify DIO group output image and write to physical outputs.
		DEBIwrite( c, WRDOut[group], c->DOutImage[group] );

		// End critical segment.
		S626MOD_CriticalEnd( c->Board );
	}
}

/////////////////////////////////////////////////////////////////////
// Return/set the DIO edge polarity to use for event captures.

U16 S626CORE_DIOGroupEdgeGet( HBD bd, U16 group )
{
	struct CORE_OBJ *c;
	U16				retval;

	// Abort if board number is not valid.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Abort if illegal DIO group number specified.
	ValidateParmRtnval( group, NUM_DIOBANKS );

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	// Return edge polarities for target DIO group.
	retval = DEBIread( c, RDEdgSel[group] );

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );

	return retval;
}

void S626CORE_DIOGroupEdgeSet( HBD bd, U16 group, U16 value )
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal DIO group number specified.
		ValidateParm( group, NUM_DIOBANKS );

		// Start critical segment.
		S626MOD_CriticalBegin( c->Board );

		// Set edge polarities for target DIO group.
		DEBIwrite( c, WREdgSel[group], value );

		// End critical segment.
		S626MOD_CriticalEnd( c->Board );
	}
}

U16 S626CORE_DIOChanEdgeGet( HBD bd, U16 chan )
{
	struct CORE_OBJ *c;
	U16				retval;

	// Abort if board number is not valid.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Abort if illegal DIO chan number specified.
	ValidateParmRtnval( chan, NUM_DIOEXTCHANS );

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	// Return edge polarity.
	retval = ( DEBIread( c, RDEdgSel[chan >> 4] ) >> ( chan & 15 ) ) & 1 ;

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );

	return retval;
}

void S626CORE_DIOChanEdgeSet( HBD bd, U16 chan, U16 value )
{
	U16 group;
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal DIO chan number specified.
		ValidateParm( chan, NUM_DIOEXTCHANS );

		// Cache the channel's group number.
		group = chan >> 4;

		// Start critical segment.
		S626MOD_CriticalBegin( c->Board );

		// Set edge polarity.
		DEBIwrite( c, WREdgSel[group], DIOBitInsert( DEBIread( c,RDEdgSel[group]), chan, value ) );

		// End critical segment.
		S626MOD_CriticalEnd( c->Board );
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// enab/disable DIO edge event capture for a group of DIO channels.
//   value  : Logic "1" in a bit position specifies that the corresponding channel's event
//            capture enable is to be modified.
//   enab : Specifies whether modified channels are to be enabled (non-zero) or disabled (zero).

void S626CORE_DIOGroupCapEnableSet( HBD bd, U16 group, U16 value, U16 enab )
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal DIO group number specified.
		ValidateParm( group, NUM_DIOBANKS );

		// Start critical segment.
		S626MOD_CriticalBegin( c->Board );

		// Prepare to treat the following data as capture enable/disable.
		DEBIwrite( c, LP_MISC1, (U16)( enab ? MISC1_EDCAP : MISC1_NOEDCAP ) );

		// Write the capture enables/disables.
		DEBIwrite( c, WRCapSel[group], value );

		// End critical segment.
		S626MOD_CriticalEnd( c->Board );
	}
}

//////////////////////////////////////////////////////////////////////////////////
// Return DIO capture enables for a group of DIO channels.  A "1" in a bit
// position indicates edge capture is enabled for the corresponding channel.

U16 S626CORE_DIOGroupCapEnableGet( HBD bd, U16 group )
{
	struct CORE_OBJ *c;
	U16				retval;

	// Abort if board number is not valid.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Abort if illegal DIO group number specified.
	ValidateParmRtnval( group, NUM_DIOBANKS );

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	// Return capture enables for target DIO group.
	retval = DEBIread( c, RDCapSel[group] );

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );

	return retval;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// enab/disable DIO edge event capture for DIO channel.

void S626CORE_DIOChanCapEnableSet( HBD bd, U16 chan, U16 enab )
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal DIO group number specified.
		ValidateParm( chan, NUM_DIOEXTCHANS );

		// Start critical segment.
		S626MOD_CriticalBegin( c->Board );

		// Prepare to treat the following data as capture enable/disable.
		DEBIwrite( c, LP_MISC1, (U16)( enab ? MISC1_EDCAP : MISC1_NOEDCAP ) );

		// Write the capture enables/disables.
		DEBIwrite( c, WRCapSel[chan >> 4], (U16)( 1 << ( chan & 15 ) ) );

		// End critical segment.
		S626MOD_CriticalEnd( c->Board );
	}
}

//////////////////////////////////////////////////////////////////////////////////
// Return capture enable for a DIO channel.

U16 S626CORE_DIOChanCapEnableGet( HBD bd, U16 chan )
{
	struct CORE_OBJ *c;
	U16				retval;

	// Abort if board number is not valid.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Abort if illegal DIO chan number specified.
	ValidateParmRtnval( chan, NUM_DIOEXTCHANS);

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	// Return capture enable.
	retval = DEBIread( c, RDCapSel[chan >> 4] );

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );

	return retval;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Clear event captures for a DIO group.  A logic "1" in a DIO's bit position in value specifies
// that the DIO's event capture is to be reset by first disabling, then re-enabling event captures.

void S626CORE_DIOGroupCapReset( HBD bd, U16 group, U16 value )
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal DIO group number specified.
		ValidateParm( group, NUM_DIOBANKS );

		// Start critical segment.
		S626MOD_CriticalBegin( c->Board );

		// Disable the specified captures.
		DEBIwrite( c, LP_MISC1, MISC1_NOEDCAP );
		DEBIwrite( c, WRCapSel[group], value );

		// Re-enable the specified captures.
		DEBIwrite( c, LP_MISC1, MISC1_EDCAP );
		DEBIwrite( c, WRCapSel[group], value );

		// End critical segment.
		S626MOD_CriticalEnd( c->Board );
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Clear event captures for a DIO channel.

void S626CORE_DIOChanCapReset( HBD bd, U16 chan )
{
	register U16 group;
	register U16 Mask;
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal DIO chan number specified.
		ValidateParm( chan, NUM_DIOEXTCHANS );

		// Cache the group number and channel mask.
		group = chan >> 4;
		Mask = 1 << ( chan & 15 );

		// Start critical segment.
		S626MOD_CriticalBegin( c->Board );

		// Disable the capture.
		DEBIwrite( c, LP_MISC1, MISC1_NOEDCAP );
		DEBIwrite( c, WRCapSel[group], Mask );

		// Re-enable the capture.
		DEBIwrite( c, LP_MISC1, MISC1_EDCAP );
		DEBIwrite( c, WRCapSel[group], Mask );

		// End critical segment.
		S626MOD_CriticalEnd( c->Board );
	}
}

////////////////////////////////////////////////////////////////////////////////////////
// Return DIO capture flags for a group of DIO channels.  A "1" in a bit position
// indicates that a capture has occured on the corresponding channel.

U16 S626CORE_DIOGroupCapStatus( HBD bd, U16 group )
{
	struct CORE_OBJ *c;
	U16				retval;

	// Abort if board number is not valid.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Abort if illegal DIO group number specified.
	ValidateParmRtnval( group, NUM_DIOBANKS );

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	// Return event capture flags for target DIO group.
	retval = DEBIread( c, RDCapFlg[group] );

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );

	return retval;
}

////////////////////////////////////////////////////////////////////////////////////////
// Return DIO capture flag for a DIO channel.

U16 S626CORE_DIOChanCapStatus( HBD bd, U16 chan )
{
	struct CORE_OBJ *c;
	U16				retval;

	// Abort if board number is not valid.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Abort if illegal DIO chan number specified.
	ValidateParmRtnval( chan, NUM_DIOEXTCHANS );

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	// Return event capture flag.
	retval = ( DEBIread( c, RDCapFlg[chan >> 4] ) >> ( chan & 15 ) ) & 1;

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );

	return retval;
}

/////////////////////////////////////////////////////////////////////////////////////
// enab/disable interrupts for a group of DIO channels.  A "1" in a bit position
// enables interrupts for the corresponding channel.

U16 S626CORE_DIOGroupIntEnableGet( HBD bd, U16 group)
{
	struct CORE_OBJ *c;
	U16				retval;

	// Abort if board number is not valid.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Abort if illegal DIO group number specified.
	ValidateParmRtnval( group, NUM_DIOBANKS );

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	// Return interrupt enables for target DIO group.
	retval = DEBIread( c, RDIntSel[group] );

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );

	return retval;
}

void S626CORE_DIOGroupIntEnableSet( HBD bd, U16 group, U16 value)
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal DIO group number specified.
		ValidateParm( group, NUM_DIOBANKS );

		// Start critical segment.
		S626MOD_CriticalBegin( c->Board );

		// Set interrupt enables for target DIO group.
		DEBIwrite( c, WRIntSel[group], value );

		// End critical segment.
		S626MOD_CriticalEnd( c->Board );
	}
}

/////////////////////////////////////////////////////////////////////////////////////
// enab/disable interrupt for a DIO channel.

U16 S626CORE_DIOChanIntEnableGet( HBD bd, U16 chan )
{
	struct CORE_OBJ *c;
	U16				retval;

	// Abort if board number is not valid.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Abort if illegal DIO chan number specified.
	ValidateParmRtnval( chan, NUM_DIOEXTCHANS );

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	// Return interrupt enable.
	retval = ( DEBIread( c, RDIntSel[chan >> 4] ) >> ( chan & 15 ) ) & 1;

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );

	return retval;
}

void S626CORE_DIOChanIntEnableSet( HBD bd, U16 chan, U16 value )
{
	register U16 group;
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal DIO chan number specified.
		ValidateParm( chan, NUM_DIOEXTCHANS );

		// Cache the group number.
		group = chan >> 4;

		// Start critical segment.
		S626MOD_CriticalBegin( c->Board );

		// Set interrupt enable.
		DEBIwrite( c, WRIntSel[group], DIOBitInsert( DEBIread( c,RDIntSel[group]), chan, value ) );

		// End critical segment.
		S626MOD_CriticalEnd( c->Board );
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return/set operating mode of specified DIO, which can function either as a DIO or as
// counter number DIOnum's overflow output.  DIOnum must be in the range 0 to 5.

#define NUM_SPECIAL_DIOS	6

U16 S626CORE_DIOChanModeGet( HBD bd, U16 DIOnum )
{
	struct CORE_OBJ *c;
	U16				retval;

	// Abort if board number is not valid.
	if ( ( c = GetBoardObject( bd ) ) == 0 )
		return 0;

	// Abort if illegal DIO channel number specified.
	ValidateParmRtnval( DIOnum, NUM_SPECIAL_DIOS );

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	// Return operating mode of target DIO.
	retval = WRMISC2image( c ) & ( 1 << ( DIOnum + 4 ) );

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );

	return retval;
}

void S626CORE_DIOChanModeSet( HBD bd, U16 DIOnum, U16 value )
{
	U16 bitmask;
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal DIO channel number specified.
		ValidateParm( DIOnum, NUM_SPECIAL_DIOS );

		// Construct bit mask that will be used to generate new image of MISC2 register.
		bitmask = 1 << ( DIOnum + 4 );

		// Start critical segment.
		S626MOD_CriticalBegin( c->Board );

		// Construct new image of MISC2 register and write it to MISC2.
		WriteMISC2( c, (U16)( value ? ( WRMISC2image( c ) | bitmask ) : ( WRMISC2image( c ) & ~bitmask ) ) );

		// End critical segment.
		S626MOD_CriticalEnd( c->Board );
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////  COUNTER FUNCTIONS  /////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
// All counter functions address a specific counter by means of the "Counter" argument,
// which is a logical counter number.  The Counter argument may have any of the following
// legal values:  0=0A, 1=1A, 2=2A, 3=0B, 4=1B, 5=2B.
/////////////////////////////////////////////////////////////////////////////////////////////

// Enumerated counter operating modes specified by ClkSrc bit field in a COUNTER_SETUP.

#define CLKSRC_COUNTER			0		// Counter:  ENC_C clock, ENC_D direction.
#define CLKSRC_TIMER			2		// Timer:    SYS_C clock, direction specified by ClkPol.
#define CLKSRC_EXTENDER			3		// Extender: OVR_A clock, ENC_D direction.

// Enumerated counter clock multipliers.

#define MULT_X0				0x0003		// Supports no multipliers; fixed physical multiplier = 3.
#define MULT_X1				0x0002		// Supports multiplier x1; fixed physical multiplier = 2.
#define MULT_X2				0x0001		// Supports multipliers x1, x2; physical multipliers = 1 or 2.
#define MULT_X4				0x0000		// Supports multipliers x1, x2, x4; physical multipliers = 0, 1 or 2.

// Sanity-check limits for parameters.

#define NUM_COUNTERS		6			// Maximum valid counter logical channel number.
#define NUM_INTSOURCES		4
#define NUM_LATCHSOURCES	4
#define NUM_CLKMULTS		4
#define NUM_CLKSOURCES		4
#define NUM_CLKPOLS			2
#define NUM_INDEXPOLS		2
#define NUM_INDEXSOURCES	2
#define NUM_LOADTRIGS		4

// Bit field positions in CRA and CRB counter control registers.

										// Bit field positions in CRA:
#define CRABIT_INDXSRC_B		14		//   B index source.
#define CRABIT_CLKSRC_B			12		//   B clock source.
#define CRABIT_INDXPOL_A		11		//   A index polarity.
#define CRABIT_LOADSRC_A		 9		//   A preload trigger.
#define CRABIT_CLKMULT_A		 7		//   A clock multiplier.
#define CRABIT_INTSRC_A			 5		//   A interrupt source.
#define CRABIT_CLKPOL_A			 4		//   A clock polarity.
#define CRABIT_INDXSRC_A		 2		//   A index source.
#define CRABIT_CLKSRC_A			 0		//   A clock source.

										// Bit field positions in CRB:
#define CRBBIT_INTRESETCMD		15		//   Interrupt reset command.
#define CRBBIT_INTRESET_B		14		//   B interrupt reset enable.
#define CRBBIT_INTRESET_A		13		//   A interrupt reset enable.
#define CRBBIT_CLKENAB_A		12		//   A clock enable.
#define CRBBIT_INTSRC_B			10		//   B interrupt source.
#define CRBBIT_LATCHSRC			 8		//   A/B latch source.
#define CRBBIT_LOADSRC_B		 6		//   B preload trigger.
#define CRBBIT_CLKMULT_B		 3		//   B clock multiplier.
#define CRBBIT_CLKENAB_B		 2		//   B clock enable.
#define CRBBIT_INDXPOL_B		 1		//   B index polarity.
#define CRBBIT_CLKPOL_B			 0		//   B clock polarity.

// Bit field masks for CRA and CRB.

#define CRAMSK_INDXSRC_B		( (U16)( 3 << CRABIT_INDXSRC_B	) )
#define CRAMSK_CLKSRC_B			( (U16)( 3 << CRABIT_CLKSRC_B	) )
#define CRAMSK_INDXPOL_A		( (U16)( 1 << CRABIT_INDXPOL_A	) )
#define CRAMSK_LOADSRC_A		( (U16)( 3 << CRABIT_LOADSRC_A	) )
#define CRAMSK_CLKMULT_A		( (U16)( 3 << CRABIT_CLKMULT_A	) )
#define CRAMSK_INTSRC_A			( (U16)( 3 << CRABIT_INTSRC_A	) )
#define CRAMSK_CLKPOL_A			( (U16)( 3 << CRABIT_CLKPOL_A	) )
#define CRAMSK_INDXSRC_A		( (U16)( 3 << CRABIT_INDXSRC_A	) )
#define CRAMSK_CLKSRC_A			( (U16)( 3 << CRABIT_CLKSRC_A	) )

#define CRBMSK_INTRESETCMD		( (U16)( 1 << CRBBIT_INTRESETCMD	) )
#define CRBMSK_INTRESET_B		( (U16)( 1 << CRBBIT_INTRESET_B	) )
#define CRBMSK_INTRESET_A		( (U16)( 1 << CRBBIT_INTRESET_A	) )
#define CRBMSK_CLKENAB_A		( (U16)( 1 << CRBBIT_CLKENAB_A	) )
#define CRBMSK_INTSRC_B			( (U16)( 3 << CRBBIT_INTSRC_B	) )
#define CRBMSK_LATCHSRC			( (U16)( 3 << CRBBIT_LATCHSRC	) )
#define CRBMSK_LOADSRC_B		( (U16)( 3 << CRBBIT_LOADSRC_B	) )
#define CRBMSK_CLKMULT_B		( (U16)( 3 << CRBBIT_CLKMULT_B	) )
#define CRBMSK_CLKENAB_B		( (U16)( 1 << CRBBIT_CLKENAB_B	) )
#define CRBMSK_INDXPOL_B		( (U16)( 1 << CRBBIT_INDXPOL_B	) )
#define CRBMSK_CLKPOL_B			( (U16)( 1 << CRBBIT_CLKPOL_B	) )

#define CRBMSK_INTCTRL			( CRBMSK_INTRESETCMD | CRBMSK_INTRESET_A | CRBMSK_INTRESET_B )	// Interrupt reset control bits.

// Bit field positions for standardized SETUP structure.

#define STDBIT_INTSRC			13
#define STDBIT_LATCHSRC			11
#define STDBIT_LOADSRC			 9
#define STDBIT_INDXSRC			 7
#define STDBIT_INDXPOL			 6
#define STDBIT_CLKSRC			 4
#define STDBIT_CLKPOL			 3
#define STDBIT_CLKMULT			 1
#define STDBIT_CLKENAB			 0

// Bit field masks for standardized SETUP structure.

#define STDMSK_INTSRC			( (U16)( 3 << STDBIT_INTSRC   ) )
#define STDMSK_LATCHSRC			( (U16)( 3 << STDBIT_LATCHSRC ) )
#define STDMSK_LOADSRC			( (U16)( 3 << STDBIT_LOADSRC  ) )
#define STDMSK_INDXSRC			( (U16)( 1 << STDBIT_INDXSRC  ) )
#define STDMSK_INDXPOL			( (U16)( 1 << STDBIT_INDXPOL  ) )
#define STDMSK_CLKSRC			( (U16)( 3 << STDBIT_CLKSRC   ) )
#define STDMSK_CLKPOL			( (U16)( 1 << STDBIT_CLKPOL   ) )
#define STDMSK_CLKMULT			( (U16)( 3 << STDBIT_CLKMULT  ) )
#define STDMSK_CLKENAB			( (U16)( 1 << STDBIT_CLKENAB  ) )

// Forward declarations for functions that are common to both A and B counters:

static U16	GetClkMult		( struct CNTR_OBJ * );
static U16	GetClkPol		( struct CNTR_OBJ * );
static U16	GetClkSrc		( struct CNTR_OBJ * );
static U16	GetIndexPol		( struct CNTR_OBJ * );
static U16	GetIndexSrc		( struct CNTR_OBJ * );
static U16	GetLatchSource	( struct CNTR_OBJ * );
static void	SetLatchSource	( struct CNTR_OBJ *, U16 value );
static U32	ReadLatch		( struct CNTR_OBJ * );
static void	Preload			( struct CNTR_OBJ *, U32 value );
static void	SetClkMult		( struct CNTR_OBJ *, U16 value );
static void	SetClkPol		( struct CNTR_OBJ *, U16 value );
static void	SetClkSrc		( struct CNTR_OBJ *, U16 value );
static void	SetIndexPol		( struct CNTR_OBJ *, U16 value );
static void	SetIndexSrc		( struct CNTR_OBJ *, U16 value );


///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////  CLS626 COUNTER FUNCTIONS  //////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
// Return overflow and index capture flags for all counter channels.

U16 S626CORE_CounterCapStatus( HBD bd )
{
	struct CORE_OBJ *c;
	U16 rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Fetch capture flags.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			rtnval = DEBIread( c, LP_RDMISC2 ) & 0xFFF0;

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}

	return rtnval;
}

//////////////////////////////////////////////////////////////////////////
// Return overflow capture status for a specific counter channel.

U16 S626CORE_CounterOverCapGet( HBD bd, U16 chan ) 
{
	struct CORE_OBJ	*c;
	U16				rtnval = 0;
	U16				mask = 1 << ( (chan << 1) + ( (chan > 2) ? 5 : 10 ) );

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Fetch overflow status.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			rtnval = ( ( ( DEBIread( c, LP_RDMISC2) & 0xFFF0 ) & mask ) != 0 );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}

	return rtnval;
}

//////////////////////////////////////////////////////////////////////////
// Return index capture status for a specific counter channel.

U16 S626CORE_CounterIndexCapGet( HBD bd, U16 chan ) 
{
	struct CORE_OBJ	*c;
	U16				rtnval = 0;
	U16				mask = 1 << ( 4 + (chan << 1) - ( (chan > 2) ? 5 : 0 ) );

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			rtnval = ( ( ( DEBIread( c, LP_RDMISC2) & 0xFFF0 ) & mask ) != 0 );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}

	return rtnval;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Return counter setup in a format (COUNTER_SETUP) that is consistent for both A and B counters.

U16 S626CORE_CounterModeGet( HBD bd, U16 chan )
{
	struct CORE_OBJ	*c;
	struct CNTR_OBJ	*k;
	U16				rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParmRtnval( chan, NUM_COUNTERS );

		// Return standardized counter setup info.
		if ( c->IsOpen )
		{
			k = &c->Counters[chan];
			rtnval = k->GetMode( k );
		}
	}

	return rtnval;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Set the operating mode for the specified counter.  Setup is treated as a COUNTER_SETUP.

void S626CORE_CounterModeSet( HBD bd, U16 chan, U16 setup )
{
	struct CORE_OBJ *c;
	struct CNTR_OBJ *k;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameter specified.
		ValidateParm( chan, NUM_COUNTERS );

		// Program counter operating mode.
		if ( c->IsOpen )
		{
			k = &c->Counters[chan];
			k->SetMode( k, setup, TRUE );
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Return/set a counter's enable.  enab: 0=always enabled, NOT_0=enabled by index.

void S626CORE_CounterEnableSet( HBD bd, U16 chan, U16 enab )
{
	struct CORE_OBJ *c;
	struct CNTR_OBJ *k;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParm( chan, NUM_COUNTERS );

		// Set the counter's enable if the board is open.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			k = &c->Counters[chan];
			k->SetEnable( k, (U16)( enab != 0 ) );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}

U16 S626CORE_CounterEnableGet( HBD bd, U16 chan )
{
	struct CORE_OBJ	*c;
	struct CNTR_OBJ	*k;
	U16				rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParmRtnval( chan, NUM_COUNTERS );

		// Fetch the counter channel's enable/disable.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			k = &c->Counters[chan];
			rtnval = k->GetEnable( k );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}

	return rtnval;
}

/////////////////////////////////////////////////////////////////
// Latch and read the counts from the specified counter.

DWORD S626CORE_CounterReadLatch( HBD bd, U16 chan )
{
	struct CORE_OBJ *c;
	U32 rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParmRtnval( chan, NUM_COUNTERS );

		// Return latched counts.
		if ( c->IsOpen )
			rtnval = ReadLatch( &c->Counters[chan] );
	}

	return rtnval;
}

//////////////////////////////////////////////////////////////////////////////
// Reset the specified counter's index and overflow event capture flags.

void S626CORE_CounterResetCapFlags( HBD bd, U16 chan )
{
	struct CORE_OBJ *c;
	struct CNTR_OBJ *k;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParm( chan, NUM_COUNTERS );

		// Reset counter's index and overflow capture flags.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			k = &c->Counters[chan];
			k->ResetCapFlags( k );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// Return/set the interrupt source for the specified counter channel.
// IntSource:  0=Disabled, 1=OverflowOnly, 2=IndexOnly, 3=IndexAndOverflow.

void S626CORE_CounterIntSourceSet( HBD bd, U16 chan, U16 IntSource )
{
	struct CORE_OBJ *c;
	struct CNTR_OBJ *k;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParm( chan, NUM_COUNTERS );
		ValidateParm( IntSource, NUM_INTSOURCES );

		// Program interrupt source.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			k = &c->Counters[chan];
			k->SetIntSrc( k, IntSource );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}

U16 S626CORE_CounterIntSourceGet( HBD bd, U16 chan )
{
	struct CORE_OBJ		*c;
	struct CNTR_OBJ	*k;
	U16					rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParmRtnval( chan, NUM_COUNTERS );

		// Return interrupt source.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			k = &c->Counters[chan];
			rtnval = k->GetIntSrc( k );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}

	return rtnval;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Return/set the latch trigger source of the specified counter.
// 0: On_read_access, 1: A_index_latches_A, 2: B_index_latches_B, 3: A_overflow_latches_B.

void S626CORE_CounterLatchSrcSet( HBD bd, U16 chan, U16 value )
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParm( chan, NUM_COUNTERS );
		ValidateParm( value, NUM_LATCHSOURCES );

		// Program the counter's latch source.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			SetLatchSource( &c->Counters[chan], value );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}

U16 S626CORE_CounterLatchSrcGet( HBD bd, U16 chan )
{
	struct CORE_OBJ *c;
	U16 rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParmRtnval( chan, NUM_COUNTERS );

		// Return the counter's latch source.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			rtnval = GetLatchSource( &c->Counters[chan] );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}

	return rtnval;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Return/set the clock multiplier.

void S626CORE_CounterClkMultSet( HBD bd, U16 chan, U16 value ) 
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParm( chan, NUM_COUNTERS );
		ValidateParm( value, NUM_CLKMULTS );

		// Program the counter's clock multiplier.
		if ( c->IsOpen )
			SetClkMult( &c->Counters[chan], value );
	}
}

U16 S626CORE_CounterClkMultGet( HBD bd, U16 chan ) 
{
	struct CORE_OBJ *c;
	U16 rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParmRtnval( chan, NUM_COUNTERS );

		// Return the counter's clock multiplier.
		if ( c->IsOpen )
			rtnval = GetClkMult( &c->Counters[chan] );
	}

	return rtnval;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Return/set the clock source.

void S626CORE_CounterClkSrcSet( HBD bd, U16 chan, U16 value ) 
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParm( chan, NUM_COUNTERS );
		ValidateParm( value, NUM_CLKSOURCES );

		// Program the counter's clock source.
		if ( c->IsOpen )
			SetClkSrc( &c->Counters[chan], value );
	}
}

U16 S626CORE_CounterClkSrcGet( HBD bd, U16 chan ) 
{
	struct CORE_OBJ *c;
	U16 rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParmRtnval( chan, NUM_COUNTERS );

		// Return the counter's clock source.
		if ( c->IsOpen )
			rtnval = GetClkSrc( &c->Counters[chan] );
	}

	return rtnval;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Return/set the clock polarity.

void S626CORE_CounterClkPolSet( HBD bd, U16 chan, U16 value ) 
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParm( chan, NUM_COUNTERS );
		ValidateParm( value, NUM_CLKPOLS );

		// Program the counter's clock polarity.
		if ( c->IsOpen )
			SetClkPol( &c->Counters[chan], value );
	}
}

U16 S626CORE_CounterClkPolGet( HBD bd, U16 chan ) 
{
	struct CORE_OBJ *c;
	U16 rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParmRtnval( chan, NUM_COUNTERS );

		// Return the counter's clock polarity.
		if ( c->IsOpen )
			rtnval = GetClkPol( &c->Counters[chan] );
	}

	return rtnval;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Return/set the index polarity.

void S626CORE_CounterIndexPolSet( HBD bd, U16 chan, U16 value )
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParm( chan, NUM_COUNTERS );
		ValidateParm( value, NUM_INDEXPOLS );

		// Program the counter's index polarity.
		if ( c->IsOpen )
			SetIndexPol( &c->Counters[chan], value );
	}
}

U16 S626CORE_CounterIndexPolGet( HBD bd, U16 chan )
{
	struct CORE_OBJ *c;
	U16 rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParmRtnval( chan, NUM_COUNTERS );

		// Return the counter's index polarity.
		if ( c->IsOpen )
			rtnval = GetIndexPol( &c->Counters[chan] );
	}

	return rtnval;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Return/set the index source.

void S626CORE_CounterIndexSrcSet( HBD bd, U16 chan, U16 value )
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParm( chan, NUM_COUNTERS );
		ValidateParm( value, NUM_INDEXSOURCES );

		// Program the counter's index source.
		if ( c->IsOpen )
			SetIndexSrc( &c->Counters[chan], value );
	}
}

U16 S626CORE_CounterIndexSrcGet( HBD bd, U16 chan )
{
	struct CORE_OBJ *c;
	U16 rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParmRtnval( chan, NUM_COUNTERS );

		// Return the counter's index source.
		if ( c->IsOpen )
			rtnval = GetIndexSrc( &c->Counters[chan] );
	}

	return rtnval;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Return/set the source that will trigger transfer of the preload register into the counter.
// 0=ThisCntr_Index, 1=ThisCntr_Overflow, 2=OverflowA (B counters only), 3=disabled.

void S626CORE_CounterLoadTrigSet( HBD bd, U16 chan, U16 Trig )
{
	struct CORE_OBJ *c;
	struct CNTR_OBJ *k;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParm( chan, NUM_COUNTERS );
		ValidateParm( Trig, NUM_LOADTRIGS );

		// Program new trigger source for counter.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			k = &c->Counters[chan];
			k->SetLoadTrig( k, Trig );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}
}

U16 S626CORE_CounterLoadTrigGet( HBD bd, U16 chan )
{
	struct CORE_OBJ	*c;
	struct CNTR_OBJ	*k;
	U16				rtnval = 0;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParmRtnval( chan, NUM_COUNTERS );

		// Get counter's trigger source.
		if ( c->IsOpen )
		{
			// Start critical segment.
			S626MOD_CriticalBegin( c->Board );

			k = &c->Counters[chan];
			rtnval = k->GetLoadTrig( k );

			// End critical segment.
			S626MOD_CriticalEnd( c->Board );
		}
	}

	return rtnval;
}

///////////////////////////////////////////////////////////////////
// Generate an index pulse.

void S626CORE_CounterSoftIndex( HBD bd, U16 chan )
{
	struct CORE_OBJ *c;
	struct CNTR_OBJ *k;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParm( chan, NUM_COUNTERS );

		// Toggle the appropriate index to produce an index pulse.
		if ( c->IsOpen )
		{
			k = &c->Counters[chan];
			k->PulseIndex( k );
		}
	}
}

/////////////////////////////////////////////////////////////////////
// Write value into specified counter's preload register.

void S626CORE_CounterPreload( HBD bd, U16 chan, U32 value )
{
	struct CORE_OBJ *c;

	// If board number is valid ...
	if ( ( c = GetBoardObject( bd ) ) != 0 )
	{
		// Abort if illegal parameters specified.
		ValidateParm( chan, NUM_COUNTERS );

		// Write new value to counter preload register.
		if ( c->IsOpen )
			Preload( &c->Counters[chan], value );
	}
}

///////////////////////////////////////////////////////////////////////////////////
/////////////////////////// PRIVATE COUNTER FUNCTIONS  ////////////////////////////
///////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////
// Read a counter's output latch.

static U32 ReadLatch( struct CNTR_OBJ *k )
{
	register U32		value;
	struct CORE_OBJ		*c = k->MyBoard;

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	// Latch counts and fetch LSW of latched counts value.
	value = (U32)DEBIread( c, k->MyLatchLsw );

	// Fetch MSW of latched counts and combine with LSW.
	value |= ( (U32) DEBIread( c, k->MyLatchLsw + 2 ) << 16 );

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );
	
	// Return latched counts.
	return value;
}

///////////////////////////////////////////////////////////////////
// Reset a counter's index and overflow event capture flags.

static void ResetCapFlags_A( struct CNTR_OBJ *k )
{
	struct CORE_OBJ *c = k->MyBoard;
	DEBIreplace( c, k->MyCRB, (U16)( ~CRBMSK_INTCTRL ), CRBMSK_INTRESETCMD | CRBMSK_INTRESET_A );
}

static void ResetCapFlags_B( struct CNTR_OBJ *k )
{
	struct CORE_OBJ *c = k->MyBoard;
	DEBIreplace( c, k->MyCRB, (U16)( ~CRBMSK_INTCTRL ), CRBMSK_INTRESETCMD | CRBMSK_INTRESET_B );
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Return counter setup in a format (COUNTER_SETUP) that is consistent for both A and B counters.

static U16 GetMode_A( struct CNTR_OBJ *k )
{
	register U16	cra;
	register U16	crb;
	register U16	setup;
	struct CORE_OBJ	*c = k->MyBoard;

	// Fetch CRA and CRB register images.
	S626MOD_CriticalBegin( c->Board );
	cra = DEBIread( c, k->MyCRA );
	crb = DEBIread( c, k->MyCRB );
	S626MOD_CriticalEnd( c->Board );

	// Populate the standardized counter setup bit fields.  Note: IndexSrc is restricted to ENC_X or IndxPol.
	setup = ( ( cra & STDMSK_LOADSRC )														// LoadSrc  = LoadSrcA.
		| ( ( crb << ( STDBIT_LATCHSRC  - CRBBIT_LATCHSRC        ) ) & STDMSK_LATCHSRC )	// LatchSrc = LatchSrcA.
		| ( ( cra << ( STDBIT_INTSRC    - CRABIT_INTSRC_A        ) ) & STDMSK_INTSRC   )	// IntSrc   = IntSrcA.
		| ( ( cra << ( STDBIT_INDXSRC   - (CRABIT_INDXSRC_A + 1) ) ) & STDMSK_INDXSRC  )	// IndxSrc  = IndxSrcA<1>.
		| ( ( cra >> ( CRABIT_INDXPOL_A - STDBIT_INDXPOL         ) ) & STDMSK_INDXPOL  )	// IndxPol  = IndxPolA.
		| ( ( crb >> ( CRBBIT_CLKENAB_A - STDBIT_CLKENAB         ) ) & STDMSK_CLKENAB  ) );	// ClkEnab  = ClkEnabA.

	// Adjust mode-dependent parameters.
	if ( cra & ( 2 << CRABIT_CLKSRC_A ) )													// If Timer mode (ClkSrcA<1> == 1):
		setup |= ( ( CLKSRC_TIMER << STDBIT_CLKSRC )										//   Indicate Timer mode.
			| ( ( cra << ( STDBIT_CLKPOL - CRABIT_CLKSRC_A ) ) & STDMSK_CLKPOL )			//   Set ClkPol to indicate count direction (ClkSrcA<0>).
			| ( MULT_X1 << STDBIT_CLKMULT ) );												//   ClkMult must be 1x in Timer mode.

	else 																					// If Counter mode (ClkSrcA<1> == 0):
		setup |= ( ( CLKSRC_COUNTER << STDBIT_CLKSRC )										//   Indicate Counter mode.
			| ( ( cra >> ( CRABIT_CLKPOL_A - STDBIT_CLKPOL ) ) & STDMSK_CLKPOL )			//   Pass through ClkPol.
			| ( ( ( cra & CRAMSK_CLKMULT_A ) == ( MULT_X0 << CRABIT_CLKMULT_A ) ) ?			//   Force ClkMult to 1x if not legal, else pass through.
				( MULT_X1 << STDBIT_CLKMULT ) :
				( ( cra >> ( CRABIT_CLKMULT_A - STDBIT_CLKMULT ) ) & STDMSK_CLKMULT ) ) );

	// Return adjusted counter setup.
	return setup;
}

static U16 GetMode_B( struct CNTR_OBJ *k )
{
	register U16	cra;
	register U16	crb;
	register U16	setup;
	struct CORE_OBJ *c = k->MyBoard;

	// Fetch CRA and CRB register images.
	S626MOD_CriticalBegin( c->Board );
	cra = DEBIread( c, k->MyCRA );
	crb = DEBIread( c, k->MyCRB );
	S626MOD_CriticalEnd( c->Board );

	// Populate the standardized counter setup bit fields.  Note: IndexSrc is restricted to ENC_X or IndxPol.
	setup = 
		( ( ( crb << ( STDBIT_INTSRC          - CRBBIT_INTSRC_B  ) ) & STDMSK_INTSRC   )	// IntSrc   = IntSrcB.
		| ( ( crb << ( STDBIT_LATCHSRC        - CRBBIT_LATCHSRC  ) ) & STDMSK_LATCHSRC )	// LatchSrc = LatchSrcB.
		| ( ( crb << ( STDBIT_LOADSRC         - CRBBIT_LOADSRC_B ) ) & STDMSK_LOADSRC  )	// LoadSrc  = LoadSrcB.
		| ( ( crb << ( STDBIT_INDXPOL         - CRBBIT_INDXPOL_B ) ) & STDMSK_INDXPOL  )	// IndxPol  = IndxPolB.
		| ( ( crb >> ( CRBBIT_CLKENAB_B       - STDBIT_CLKENAB   ) ) & STDMSK_CLKENAB  )	// ClkEnab  = ClkEnabB.
		| ( ( cra >> ( (CRABIT_INDXSRC_B + 1) - STDBIT_INDXSRC   ) ) & STDMSK_INDXSRC  ) );	// IndxSrc  = IndxSrcB<1>.

	// Adjust mode-dependent parameters.
	if ( ( crb & CRBMSK_CLKMULT_B ) == ( MULT_X0 << CRBBIT_CLKMULT_B ) )					// If Extender mode (ClkMultB == MULT_X0):
		setup |= ( ( CLKSRC_EXTENDER << STDBIT_CLKSRC )										//   Indicate Extender mode.
			| ( MULT_X1 << STDBIT_CLKMULT )													//   Indicate multiplier is 1x.
			| ( ( cra >> ( CRABIT_CLKSRC_B - STDBIT_CLKPOL ) ) & STDMSK_CLKPOL ) );			//   Set ClkPol equal to Timer count direction (ClkSrcB<0>).

	else if ( cra & ( 2 << CRABIT_CLKSRC_B ) )												// If Timer mode (ClkSrcB<1> == 1):
		setup |= ( ( CLKSRC_TIMER << STDBIT_CLKSRC )										//   Indicate Timer mode.
			| ( MULT_X1 << STDBIT_CLKMULT )													//   Indicate multiplier is 1x.
			| ( ( cra >> ( CRABIT_CLKSRC_B - STDBIT_CLKPOL ) ) & STDMSK_CLKPOL ) );			//   Set ClkPol equal to Timer count direction (ClkSrcB<0>).

	else																					// If Counter mode (ClkSrcB<1> == 0):
		setup |= ( ( CLKSRC_COUNTER << STDBIT_CLKSRC )										//   Indicate Timer mode.
			| ( ( crb >> ( CRBBIT_CLKMULT_B - STDBIT_CLKMULT ) ) & STDMSK_CLKMULT )			//   Clock multiplier is passed through.	
			| ( ( crb << ( STDBIT_CLKPOL - CRBBIT_CLKPOL_B ) ) & STDMSK_CLKPOL ) );			//   Clock polarity is passed through.	

	// Return adjusted counter setup.
	return setup;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Set the operating mode for the specified counter.  The setup parameter is treated as
// a COUNTER_SETUP data type.  The following parameters are programmable (all other parms
// are ignored):  ClkMult, ClkPol, ClkEnab, IndexSrc, IndexPol, LoadSrc.

static void SetMode_A( struct CNTR_OBJ *k, U16 Setup, U16 DisableIntSrc )
{
	register U16	cra;	
	register U16	crb;
	register U16	setup = Setup;		// Cache the Standard Setup.
	struct CORE_OBJ	*c = k->MyBoard;

	// Initialize CRA and CRB images.
	cra = (	( setup & CRAMSK_LOADSRC_A )													// Preload trigger is passed through.
		| ( ( setup & STDMSK_INDXSRC ) >> ( STDBIT_INDXSRC - (CRABIT_INDXSRC_A + 1) ) ) );	// IndexSrc is restricted to ENC_X or IndxPol.
	
	crb = ( CRBMSK_INTRESETCMD | CRBMSK_INTRESET_A											// Reset any pending CounterA event captures.
		| ( ( setup & STDMSK_CLKENAB ) << ( CRBBIT_CLKENAB_A - STDBIT_CLKENAB ) ) );		// Clock enable is passed through.
	
	// Force IntSrc to Disabled if DisableIntSrc is asserted.
	if ( !DisableIntSrc )
		cra |= ( ( setup & STDMSK_INTSRC ) >> ( STDBIT_INTSRC - CRABIT_INTSRC_A ) );

	// Populate all mode-dependent attributes of CRA & CRB images.
	switch ( ( setup & STDMSK_CLKSRC ) >> STDBIT_CLKSRC )
	{
	case CLKSRC_EXTENDER:																	// Extender Mode: Force to Timer mode (Extender valid only for B counters).

	case CLKSRC_TIMER:																		// Timer Mode:
		cra |= ( ( 2 << CRABIT_CLKSRC_A )													//   ClkSrcA<1> selects system clock
			| ( ( setup & STDMSK_CLKPOL ) >> ( STDBIT_CLKPOL - CRABIT_CLKSRC_A ) )			//     with count direction (ClkSrcA<0>) obtained from ClkPol.
			| ( 1 << CRABIT_CLKPOL_A )														//   ClkPolA behaves as always-on clock enable.
			| ( MULT_X1 << CRABIT_CLKMULT_A ) );											//   ClkMult must be 1x.
		break;

	default:																				// Counter Mode:
		cra |= ( CLKSRC_COUNTER																//   Select ENC_C and ENC_D as clock/direction inputs.
			| ( ( setup & STDMSK_CLKPOL ) << ( CRABIT_CLKPOL_A - STDBIT_CLKPOL ) )			//   Clock polarity is passed through.
			| ( ( ( setup & STDMSK_CLKMULT ) == ( MULT_X0 << STDBIT_CLKMULT ) ) ?			//   Force multiplier to x1 if not legal, otherwise pass through.
				( MULT_X1 << CRABIT_CLKMULT_A ) :
				( ( setup & STDMSK_CLKMULT ) << ( CRABIT_CLKMULT_A - STDBIT_CLKMULT ) ) ) );
	}

	// Force positive index polarity if IndxSrc is software-driven only, otherwise pass it through.
	if ( ~setup & STDMSK_INDXSRC )
		cra |= ( ( setup & STDMSK_INDXPOL ) << ( CRABIT_INDXPOL_A - STDBIT_INDXPOL ) );

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	// If IntSrc has been forced to Disabled, update the MISC2 interrupt enable mask to indicate the counter interrupt is disabled.
	if ( DisableIntSrc )
		c->CounterIntEnabs &= ~k->MyEventBits[3];

	// While retaining CounterB and LatchSrc configurations, program the new counter operating mode.
	DEBIreplace( c, k->MyCRA, CRAMSK_INDXSRC_B | CRAMSK_CLKSRC_B, cra );
	DEBIreplace( c, k->MyCRB, (U16)( ~( CRBMSK_INTCTRL | CRBMSK_CLKENAB_A ) ), crb );

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );
}

static void SetMode_B( struct CNTR_OBJ *k, U16 Setup, U16 DisableIntSrc )
{
	register U16	cra;	
	register U16	crb;
	register U16	setup = Setup;		// Cache the Standard Setup.
	struct CORE_OBJ	*c = k->MyBoard;

	// Initialize CRA and CRB images.
	cra = (
		( setup & STDMSK_INDXSRC ) << ( (CRABIT_INDXSRC_B + 1) - STDBIT_INDXSRC ) );	// IndexSrc field is restricted to ENC_X or IndxPol.

	crb = ( CRBMSK_INTRESETCMD | CRBMSK_INTRESET_B										// Reset event captures and disable interrupts.
		| ( ( setup & STDMSK_CLKENAB ) << ( CRBBIT_CLKENAB_B - STDBIT_CLKENAB ) )		// Clock enable is passed through.
		| ( ( setup & STDMSK_LOADSRC ) >> ( STDBIT_LOADSRC - CRBBIT_LOADSRC_B ) ) );	// Preload trigger source is passed through.

	// Force IntSrc to Disabled if DisableIntSrc is asserted.
	if ( !DisableIntSrc )
		crb |= ( ( setup & STDMSK_INTSRC ) >> ( STDBIT_INTSRC - CRBBIT_INTSRC_B ) );

	// Populate all mode-dependent attributes of CRA & CRB images.
	switch ( ( setup & STDMSK_CLKSRC ) >> STDBIT_CLKSRC )
	{
	case CLKSRC_TIMER:																	// Timer Mode:
		cra |= ( ( 2 << CRABIT_CLKSRC_B )												//   ClkSrcB<1> selects system clock
			| ( ( setup & STDMSK_CLKPOL ) << ( CRABIT_CLKSRC_B - STDBIT_CLKPOL ) ) );	//     with direction (ClkSrcB<0>) obtained from ClkPol.
		crb	|= ( ( 1 << CRBBIT_CLKPOL_B )												//   ClkPolB behaves as always-on clock enable.
			| ( MULT_X1 << CRBBIT_CLKMULT_B ) );										//   ClkMultB must be 1x.
		break;

	case CLKSRC_EXTENDER:																// Extender Mode:
		cra |= ( ( 2 << CRABIT_CLKSRC_B )												//   ClkSrcB source is OverflowA (same as "timer")
			| ( ( setup & STDMSK_CLKPOL ) << ( CRABIT_CLKSRC_B - STDBIT_CLKPOL ) ) );	//     with direction obtained from ClkPol.
		crb |= ( ( 1 << CRBBIT_CLKPOL_B )												//   ClkPolB controls IndexB -- always set to active.
			| ( MULT_X0 << CRBBIT_CLKMULT_B ) );										//   ClkMultB selects OverflowA as the clock source.
		break;

	default:																			// Counter Mode:
		cra |= ( CLKSRC_COUNTER << CRABIT_CLKSRC_B );									//   Select ENC_C and ENC_D as clock/direction inputs.
		crb |= ( ( ( setup & STDMSK_CLKPOL ) >> ( STDBIT_CLKPOL - CRBBIT_CLKPOL_B ) )	//   ClkPol is passed through.
			| ( ( ( setup & STDMSK_CLKMULT ) == ( MULT_X0 << STDBIT_CLKMULT ) ) ?		//   Force ClkMult to x1 if not legal, otherwise pass through.
			( MULT_X1 << CRBBIT_CLKMULT_B ) :
			( ( setup & STDMSK_CLKMULT ) << ( CRBBIT_CLKMULT_B - STDBIT_CLKMULT ) ) ) );
	}

	// Force positive index polarity if IndxSrc is software-driven only, otherwise pass it through.
	if ( ~setup & STDMSK_INDXSRC )
		crb |= ( ( setup & STDMSK_INDXPOL ) >> ( STDBIT_INDXPOL - CRBBIT_INDXPOL_B ) );

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	// If IntSrc has been forced to Disabled, update the MISC2 interrupt enable mask to indicate the counter interrupt is disabled.
	if ( DisableIntSrc )
		c->CounterIntEnabs &= ~k->MyEventBits[3];

	// While retaining CounterA and LatchSrc configurations, program the new counter operating mode.
	DEBIreplace( c, k->MyCRA, (U16)( ~( CRAMSK_INDXSRC_B | CRAMSK_CLKSRC_B ) ), cra );
	DEBIreplace( c, k->MyCRB, CRBMSK_CLKENAB_A | CRBMSK_LATCHSRC, crb );

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );
}

//////////////////////////////////////////////////////////////////////////////////
// Return/set a counter's enable.  enab: 0=always enabled, 1=enabled by index.

static void SetEnable_A( struct CNTR_OBJ *k, U16 enab )
{
	DEBIreplace( k->MyBoard, k->MyCRB, (U16)( ~( CRBMSK_INTCTRL | CRBMSK_CLKENAB_A ) ), (U16)( enab << CRBBIT_CLKENAB_A ) );
}

static void SetEnable_B( struct CNTR_OBJ *k, U16 enab )
{
	DEBIreplace( k->MyBoard, k->MyCRB, (U16)( ~( CRBMSK_INTCTRL | CRBMSK_CLKENAB_B ) ), (U16)( enab << CRBBIT_CLKENAB_B ) );
}

static U16 GetEnable_A( struct CNTR_OBJ *k )
{
	return ( DEBIread( k->MyBoard, k->MyCRB) >> CRBBIT_CLKENAB_A ) & 1;
}

static U16 GetEnable_B( struct CNTR_OBJ *k )
{
	return ( DEBIread( k->MyBoard, k->MyCRB) >> CRBBIT_CLKENAB_B ) & 1;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Return/set a counter pair's latch trigger source.
// 0: On read access, 1: A index latches A, 2: B index latches B, 3: A overflow latches B.

static void SetLatchSource( struct CNTR_OBJ *k, U16 value )
{
	DEBIreplace( k->MyBoard, k->MyCRB, (U16)( ~( CRBMSK_INTCTRL | CRBMSK_LATCHSRC ) ), (U16)( value << CRBBIT_LATCHSRC ) );
}

static U16 GetLatchSource( struct CNTR_OBJ *k )
{
	return ( DEBIread( k->MyBoard, k->MyCRB) >> CRBBIT_LATCHSRC ) & 3;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Return/set the event that will trigger transfer of the preload register into the counter.
// 0=ThisCntr_Index, 1=ThisCntr_Overflow, 2=OverflowA (B counters only), 3=disabled.

static void SetLoadTrig_A( struct CNTR_OBJ *k, U16 Trig )
{
	DEBIreplace( k->MyBoard, k->MyCRA, (U16)( ~CRAMSK_LOADSRC_A ), (U16)( Trig << CRABIT_LOADSRC_A ) );
}

static void SetLoadTrig_B( struct CNTR_OBJ *k, U16 Trig )
{
	DEBIreplace( k->MyBoard, k->MyCRB, (U16)( ~( CRBMSK_LOADSRC_B | CRBMSK_INTCTRL ) ), (U16)( Trig << CRBBIT_LOADSRC_B ) );
}

static U16 GetLoadTrig_A( struct CNTR_OBJ *k )
{
	return ( DEBIread( k->MyBoard, k->MyCRA) >> CRABIT_LOADSRC_A ) & 3;
}

static U16 GetLoadTrig_B( struct CNTR_OBJ *k )
{
	return ( DEBIread( k->MyBoard, k->MyCRB) >> CRBBIT_LOADSRC_B ) & 3;
}

//////////////////////////////////////////////////////////////////////////////////
// Return/set counter interrupt source and clear any captured index/overflow events.
// IntSource: 0=Disabled, 1=OverflowOnly, 2=IndexOnly, 3=IndexAndOverflow.

static void SetIntSrc_A( struct CNTR_OBJ *k, U16 IntSource )
{
	struct CORE_OBJ *c = k->MyBoard;

	//S626MOD_CriticalBegin( c->Board );		// Start critical segment.	-- done at upper level.
	
	// Reset any pending counter overflow or index captures.
	DEBIreplace( c, k->MyCRB, (U16)( ~CRBMSK_INTCTRL ), CRBMSK_INTRESETCMD | CRBMSK_INTRESET_A );

	// Program counter interrupt source.
	DEBIreplace( c, k->MyCRA, ~CRAMSK_INTSRC_A, (U16)( IntSource << CRABIT_INTSRC_A ) );

	// Update MISC2 interrupt enable mask.
	c->CounterIntEnabs = ( c->CounterIntEnabs & ~k->MyEventBits[3] ) | k->MyEventBits[IntSource];

	//S626MOD_CriticalEnd( c->Board );			// End critical segment.	-- done at upper level.
}

static void SetIntSrc_B( struct CNTR_OBJ *k, U16 IntSource )
{
	U16 crb;
	struct CORE_OBJ *c = k->MyBoard;

	// Start critical segment.
	//S626MOD_CriticalBegin( c->Board );	// -- done at upper level.
	
	// Cache writeable CRB register image.
	crb = DEBIread( c, k->MyCRB ) & ~CRBMSK_INTCTRL;
	
	// Reset any pending counter overflow or index captures.
	DEBIwrite( c, k->MyCRB, (U16)( crb | CRBMSK_INTRESETCMD | CRBMSK_INTRESET_B ) );

	// Program counter interrupt source.
	DEBIwrite( c, k->MyCRB, (U16)( ( crb & ~CRBMSK_INTSRC_B ) | ( IntSource << CRBBIT_INTSRC_B ) ) );

	// Update MISC2 interrupt enable mask.
	c->CounterIntEnabs = ( c->CounterIntEnabs & ~k->MyEventBits[3] ) | k->MyEventBits[IntSource];

	// End critical segment.
	//S626MOD_CriticalEnd( c->Board );		// -- done at upper level.
}

static U16 GetIntSrc_A( struct CNTR_OBJ *k )
{
	return ( DEBIread( k->MyBoard, k->MyCRA) >> CRABIT_INTSRC_A ) & 3;
}

static U16 GetIntSrc_B( struct CNTR_OBJ *k )
{
	return ( DEBIread( k->MyBoard, k->MyCRB) >> CRBBIT_INTSRC_B ) & 3;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Return/set the clock multiplier.

static void SetClkMult( struct CNTR_OBJ *k, U16 value ) 
{
	k->SetMode( k, (U16)( ( k->GetMode( k ) & ~STDMSK_CLKMULT ) | ( value << STDBIT_CLKMULT ) ), FALSE );
}

static U16 GetClkMult( struct CNTR_OBJ *k ) 
{
	return ( k->GetMode( k ) >> STDBIT_CLKMULT ) & 3;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Return/set the clock polarity.

static void SetClkPol( struct CNTR_OBJ *k, U16 value ) 
{
	k->SetMode( k, (U16)( ( k->GetMode( k ) & ~STDMSK_CLKPOL ) | ( value << STDBIT_CLKPOL ) ), FALSE );
}

static U16 GetClkPol( struct CNTR_OBJ *k ) 
{
	return ( k->GetMode( k ) >> STDBIT_CLKPOL ) & 1;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Return/set the clock source.

static void SetClkSrc( struct CNTR_OBJ *k, U16 value ) 
{
	k->SetMode( k, (U16)( ( k->GetMode( k ) & ~STDMSK_CLKSRC ) | ( value << STDBIT_CLKSRC ) ), FALSE );
}

static U16 GetClkSrc( struct CNTR_OBJ *k ) 
{
	return ( k->GetMode( k ) >> STDBIT_CLKSRC ) & 3;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Return/set the index polarity.

static void SetIndexPol( struct CNTR_OBJ *k, U16 value ) 
{
	k->SetMode( k, (U16)( ( k->GetMode( k ) & ~STDMSK_INDXPOL ) | ( (value != 0) << STDBIT_INDXPOL ) ), FALSE );
}

static U16 GetIndexPol( struct CNTR_OBJ *k ) 
{
	return ( k->GetMode( k ) >> STDBIT_INDXPOL ) & 1;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Return/set the index source.

static void SetIndexSrc( struct CNTR_OBJ *k, U16 value ) 
{
	k->SetMode( k, (U16)( ( k->GetMode( k ) & ~STDMSK_INDXSRC ) | ( (value != 0) << STDBIT_INDXSRC ) ), FALSE );
}

static U16 GetIndexSrc( struct CNTR_OBJ *k ) 
{
	return ( k->GetMode( k ) >> STDBIT_INDXSRC ) & 1;
}

///////////////////////////////////////////////////////////////////
// Generate an index pulse.

static void PulseIndex_A( struct CNTR_OBJ *k )
{
	register U16 cra;
	struct CORE_OBJ *c = k->MyBoard;

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	cra = DEBIread( c, k->MyCRA );										// Pulse index.
	DEBIwrite( c, k->MyCRA, (U16)( cra ^ CRAMSK_INDXPOL_A ) );
	DEBIwrite( c, k->MyCRA, cra );

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );
}

static void PulseIndex_B( struct CNTR_OBJ *k )
{
	register U16 crb;
	struct CORE_OBJ *c = k->MyBoard;

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	crb = DEBIread( c, k->MyCRB ) & ~CRBMSK_INTCTRL;					// Pulse index.
	DEBIwrite( c, k->MyCRB, (U16)( crb ^ CRBMSK_INDXPOL_B ) );
	DEBIwrite( c, k->MyCRB, crb);

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );
}

/////////////////////////////////////////////////////////
// Write value into counter preload register.

static void Preload( struct CNTR_OBJ *k, U32 value )
{
	struct CORE_OBJ *c = k->MyBoard;

	// Start critical segment.
	S626MOD_CriticalBegin( c->Board );

	DEBIwrite( c, (U16)( k->MyLatchLsw     ), (U16)  value         );	// Write value to preload register.
	DEBIwrite( c, (U16)( k->MyLatchLsw + 2 ), (U16)( value >> 16 ) );

	// End critical segment.
	S626MOD_CriticalEnd( c->Board );
}

///////////////////////////////////////////////////////////////////////////////
// Counter objects constructor.

// Register address to use for PreLoad and Latched counts LSW.
static const U16 AdrsCntrLSW[] = { LP_CNTR0ALSW, LP_CNTR1ALSW, LP_CNTR2ALSW, LP_CNTR0BLSW, LP_CNTR1BLSW, LP_CNTR2BLSW };

// CRA register address to use.
static const U16 AdrsCRA[] = { LP_CR0A, LP_CR1A, LP_CR2A, LP_CR0A, LP_CR1A, LP_CR2A };

// CRB register address to use.
static const U16 AdrsCRB[] = { LP_CR0B, LP_CR1B, LP_CR2B, LP_CR0B, LP_CR1B, LP_CR2B };

// Counter overflow/index event flag masks for RDMISC2.
#define INDXMASK(C)		( 1 << ( ( (C) > 2 ) ? ( (C) * 2 - 1 ) : ( (C) * 2 +  4 ) ) )
#define OVERMASK(C)		( 1 << ( ( (C) > 2 ) ? ( (C) * 2 + 5 ) : ( (C) * 2 + 10 ) ) )
#define EVBITS(C)		{ 0, OVERMASK(C), INDXMASK(C), OVERMASK(C) | INDXMASK(C) }

// Translation table to map IntSrc into equivalent RDMISC2 event flag bits.
static const U16 EventBits[][4] = { EVBITS(0), EVBITS(1), EVBITS(2), EVBITS(3), EVBITS(4), EVBITS(5) };

// Constructor for counter objects.
static void CountersCreate( struct CORE_OBJ *c )
{
	int				i;
	int				chan;
	struct CNTR_OBJ	*k;

	for ( chan = 0; chan < 6; chan++ )
	{
		k				= &c->Counters[chan];		// Pointer to this counter object.

		k->MyBoard		= c;						// Pointer to this counter's parent core object.
		k->MyCRA		= AdrsCRA[chan];			// Address of CRA register.
		k->MyCRB		= AdrsCRB[chan];			// Address of CRB register.
		k->MyLatchLsw	= AdrsCntrLSW[chan];		// Address of Preload & Latch LSW registers.
		for ( i = 0; i < 4; i++ )
			k->MyEventBits[i] = EventBits[chan][i];	// Event flag bit overlays for RDMISC2.

		// Load vfunc pointers for the derived counter objects.
		if ( chan < 3 )
		{
			k->GetEnable		= GetEnable_A;
			k->GetIntSrc		= GetIntSrc_A;
			k->GetLoadTrig		= GetLoadTrig_A;
			k->GetMode			= GetMode_A;
			k->PulseIndex		= PulseIndex_A;
			k->SetEnable		= SetEnable_A;
			k->SetIntSrc		= SetIntSrc_A;
			k->SetLoadTrig		= SetLoadTrig_A;
			k->SetMode			= SetMode_A;
			k->ResetCapFlags	= ResetCapFlags_A;
		}
		else
		{
			k->GetEnable		= GetEnable_B;
			k->GetIntSrc		= GetIntSrc_B;
			k->GetLoadTrig		= GetLoadTrig_B;
			k->GetMode			= GetMode_B;
			k->PulseIndex		= PulseIndex_B;
			k->SetEnable		= SetEnable_B;
			k->SetIntSrc		= SetIntSrc_B;
			k->SetLoadTrig		= SetLoadTrig_B;
			k->SetMode			= SetMode_B;
			k->ResetCapFlags	= ResetCapFlags_B;
		}
	}
}

static void CountersInit( struct CORE_OBJ *c )
{
	int chan;
	struct CNTR_OBJ	*k;

	// Disable all counter interrupts and clear any captured counter events.
	for ( chan = 0; chan < 6; chan++ )
	{
		k = &c->Counters[chan];
		k->SetIntSrc( k, 0 );
	}
}
