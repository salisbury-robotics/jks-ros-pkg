//////////////////////////////////////////////////////////////
// Module    : s626core.h									//
// Function  : Global defines for 626 core object.			//
// Author    : Jim Lamberson								//
// Copyright : (C) 2003 Sensoray							//
//////////////////////////////////////////////////////////////


#ifndef DEF_626CORE
#define DEF_626CORE

#ifdef __cplusplus
extern "C" {
#endif

#include "s626types.h"
/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////  CONSTANTS  //////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

// Maximum number of 626 boards supported by the core.
#define MAX_BOARDS			4

// PCI device and vendor IDs.
#define VENDORID626			0x1131		// PCI VendorID for Philips.
#define DEVICEID626			0x7146		// PCI DeviceID for SAA7146 PCI bridge.
#define SUBVENDORID626		0x6000		// PCI SubVendorID for Sensoray.
#define SUBDEVICEID626		626			// PCI SubDeviceID for Model 626.





//typedef DWORD	HBD;
//typedef void	(* FPTR_ERR) (U32 ErrFlags);	// Pointer to application's error callback function.
//

#ifndef DEF_626MOD

typedef struct {							// DMA buffer description used by Board Class.
	void				*PhysicalBase;			//   Physical base address.
	void				*LogicalBase;			//   Logical base address.
	U32				DMAHandle;				//   Buffer ID/reference used by interface class.
} DMABUF;

#endif	// #ifndef DEF_626MOD

typedef struct {					// Used by core object to specify a register access operation to the module level.
	U32					IsWrite;				//   Boolean: TRUE = dword write, FALSE = dword read.
	U32					AdrsOS;					//   Register address offset.
	U32					DataVal;				//   Data value (applicable only for write operations).
} ACCESS_SPEC;

typedef struct {					// Specification for a sequence of register access actions.
	U32					NumActions;				//   Number of actions in list.
	ACCESS_SPEC			Actions[20];			//   List of access actions.
} ACTION_LIST;

typedef struct {					// List of actions needed to disable interrupts.
	U32					StatusAndMask;			//   AND bit mask for interrupt status flag.
	U32					StatusXorMask;			//   XOR bit mask for interrupt status flag.
	ACTION_LIST			StatusActions;			//   Actions that will retrieve the board's IRQ status.
	ACTION_LIST			ProcessActions;			//   Actions that will negate board's IRQ.
} INT_ACTIONS;


/////////////////////////////////////////////////////////////////////////////////////
////////////////////////////  CORE FUNCTION PROTOTYPES  /////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

UINT	S626CORE_CoreOpen( HBD bd, UINT num_boards );
void	S626CORE_CoreClose( HBD bd );
void	S626CORE_BoardOpen( HBD bd, U8 IsBoardRevA );
void	S626CORE_BoardClose( HBD bd );
void	S626CORE_SetErrCallback( HBD bd, FPTR_ERR Callback );
U32	S626CORE_GetErrors( HBD bd );
void	S626CORE_SetErrors( HBD bd, U32 NewFlags );
U32	S626CORE_GetAddress( HBD bd );
void	S626CORE_SetAddress( HBD bd, U32 adrs );
PVOID	S626CORE_GetBase( HBD bd );
void	S626CORE_SetBase( HBD bd, PVOID base );

U8		S626CORE_I2Cread( HBD bd, U8 addr );
void	S626CORE_I2Cwrite( HBD bd, U8 addr, U8 val );
U16		S626CORE_DEBIread( HBD bd, U16 addr );
void	S626CORE_DEBIwrite( HBD bd, U16 addr, U16 wdata );

U16		S626CORE_InterruptEnableGet( HBD bd );
void	S626CORE_InterruptEnableSet( HBD bd, U16 enab );
void	S626CORE_InterruptMask( HBD bd );
U16		S626CORE_InterruptPending( HBD bd );
void	S626CORE_InterruptStatus( HBD bd, U16 *IntFlags );

void	S626CORE_ResetADC( HBD bd, U8 *ppl );
void	S626CORE_StartADC( HBD bd );
void	S626CORE_WaitDoneADC( HBD bd, U16 *pdata );
U16		S626CORE_SingleADC( HBD bd, U16 AdcSpec );
void	S626CORE_ReadADC( HBD bd, U16 *pdata );

void	S626CORE_WriteTrimDAC( HBD bd, U8 logchan, U8 val );
U8		S626CORE_ReadTrimDAC( HBD bd, U8 logchan );
void	S626CORE_WriteDAC( HBD bd, U16 chan, S16 dacdata );
S16		S626CORE_ReadDAC( HBD bd, U16 chan );

U16		S626CORE_BackupEnableGet( HBD bd );
void	S626CORE_BackupEnableSet( HBD bd, U16 enab );
U16		S626CORE_ChargeEnableGet( HBD bd );
void	S626CORE_ChargeEnableSet( HBD bd, U16 enab );

U16		S626CORE_WatchdogTimeout( HBD bd );
U16		S626CORE_WatchdogEnableGet( HBD bd );
void	S626CORE_WatchdogEnableSet( HBD bd, U16 enab );
U16		S626CORE_WatchdogPeriodGet( HBD bd );
void	S626CORE_WatchdogPeriodSet( HBD bd, U16 value );
void	S626CORE_WatchdogReset( HBD bd );

U16		S626CORE_DIOGroupRead( HBD bd, U16 group );
U16		S626CORE_DIOChanRead( HBD bd, U16 chan );
U16		S626CORE_DIOGroupWriteGet( HBD bd, U16 group );
void	S626CORE_DIOGroupWriteSet( HBD bd, U16 group, U16 value );
U16		S626CORE_DIOChanWriteGet( HBD bd, U16 chan );
void	S626CORE_DIOChanWriteSet( HBD bd, U16 chan, U16 value );
U16		S626CORE_DIOGroupEdgeGet( HBD bd, U16 group );
void	S626CORE_DIOGroupEdgeSet( HBD bd, U16 group, U16 value );
U16		S626CORE_DIOChanEdgeGet( HBD bd, U16 chan );
void	S626CORE_DIOChanEdgeSet( HBD bd, U16 chan, U16 value );
void	S626CORE_DIOGroupCapEnableSet( HBD bd, U16 group, U16 value, U16 enab );
U16		S626CORE_DIOGroupCapEnableGet( HBD bd, U16 group );
void	S626CORE_DIOChanCapEnableSet( HBD bd, U16 chan, U16 enab );
U16		S626CORE_DIOChanCapEnableGet( HBD bd, U16 chan );
void	S626CORE_DIOGroupCapReset( HBD bd, U16 group, U16 value );
void	S626CORE_DIOChanCapReset( HBD bd, U16 chan );
U16		S626CORE_DIOGroupCapStatus( HBD bd, U16 group );
U16		S626CORE_DIOChanCapStatus( HBD bd, U16 chan );
U16		S626CORE_DIOGroupIntEnableGet( HBD bd, U16 group);
void	S626CORE_DIOGroupIntEnableSet( HBD bd, U16 group, U16 value);
U16		S626CORE_DIOChanModeGet( HBD bd, U16 DIOnum );
void	S626CORE_DIOChanModeSet( HBD bd, U16 DIOnum, U16 value );

U16		S626CORE_CounterCapStatus( HBD bd );
U16		S626CORE_CounterOverCapGet( HBD bd, U16 chan ) ;
U16		S626CORE_CounterIndexCapGet( HBD bd, U16 chan ) ;
U16		S626CORE_CounterModeGet( HBD bd, U16 chan );
void	S626CORE_CounterModeSet( HBD bd, U16 chan, U16 setup );
void	S626CORE_CounterEnableSet( HBD bd, U16 chan, U16 enab );
U16		S626CORE_CounterEnableGet( HBD bd, U16 chan );
DWORD		S626CORE_CounterReadLatch( HBD bd, U16 chan );
void	S626CORE_CounterResetCapFlags( HBD bd, U16 chan );
void	S626CORE_CounterIntSourceSet( HBD bd, U16 chan, U16 IntSource );
U16		S626CORE_CounterIntSourceGet( HBD bd, U16 chan );
void	S626CORE_CounterLatchSrcSet( HBD bd, U16 chan, U16 value );
U16		S626CORE_CounterLatchSrcGet( HBD bd, U16 chan );
void	S626CORE_CounterClkMultSet( HBD bd, U16 chan, U16 value ) ;
U16		S626CORE_CounterClkMultGet( HBD bd, U16 chan ) ;
void	S626CORE_CounterClkSrcSet( HBD bd, U16 chan, U16 value );
U16		S626CORE_CounterClkSrcGet( HBD bd, U16 chan );
void	S626CORE_CounterClkPolSet( HBD bd, U16 chan, U16 value );
U16		S626CORE_CounterClkPolGet( HBD bd, U16 chan );
void	S626CORE_CounterIndexPolSet( HBD bd, U16 chan, U16 value );
U16		S626CORE_CounterIndexPolGet( HBD bd, U16 chan );
void	S626CORE_CounterIndexSrcSet( HBD bd, U16 chan, U16 value );
U16		S626CORE_CounterIndexSrcGet( HBD bd, U16 chan );
void	S626CORE_CounterLoadTrigSet( HBD bd, U16 chan, U16 Trig );
U16		S626CORE_CounterLoadTrigGet( HBD bd, U16 chan );
void	S626CORE_CounterSoftIndex( HBD bd, U16 chan );
void	S626CORE_CounterPreload( HBD bd, U16 chan, U32 value );

#ifdef __cplusplus
}
#endif

#endif	// #ifndef DEF_626CORE







