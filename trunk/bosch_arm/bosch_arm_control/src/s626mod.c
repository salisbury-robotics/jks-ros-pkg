//////////////////////////////////////////////////////////////////////////////////////////
// Module    : s626mod.c								//
// Function  : Linux OS dependent module-level functions for Sensoray models 626	//
// Author    : Charlie X. Liu								//
// Revision  :										//
//             Feb., 2004   Charlie X. Liu   initial.					//
//             Mar., 2004   Charlie X. Liu   fixes for MOD-level object constructor and	//
//					     and S626_OpenBoard().			//
//             Dec., 2006   Charlie X. Liu   added support for using multi-626 boards.	//
//             Feb, 2007    Dean Anderson    added support for 64bit linux              //
// Copyright : (C) Sensoray 2004~2007							//
//											//
// $Log:3$										//
//											//
//////////////////////////////////////////////////////////////////////////////////////////

/*
Copyright (C) 2004  Sensoray Co., Inc.  This program is free software;
you can redistribute it and/or modify it under the terms of the GNU
General Public License as published by the Free Software Foundation;
either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

In addition, as a special exception, Sensoray Co., Inc., gives permission
to link the code of this program with the 's626core.o' library
and distribute linked combinations including the two. You must obey the
GNU General Public License in all respects for all of the code used other
than 's626core.o'. If you modify this file, you may extend this exception
to your version of the file, but you are not obligated to do so. If you
do not wish to do so, delete this exception statement from your version.
*/

// The necessary header files
#include "s626drv.h"				// interface to low-level driver
#include "App626.h"				// Model 626 application definitions.
#include "s626mod.h"				// Interface to s626mod.o
#include "s626core.h"				// Interface to s626core.o.
#include "s626.h"				// IOCTL defines
#include "s626api.h"				// API Declarations
#include "s626types.h"
#include <stdio.h>           
#include <unistd.h>          
#include <fcntl.h>           
#include <sys/types.h>       
#include <sys/stat.h>       
#include <sys/ioctl.h>
#include <pthread.h>


/////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////  CONSTANTS  ////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

// Manifest constants ---------------------------------

#define TRUE			1
#define FALSE			0

#define REGION_SIZE_626		512		// Size of SAA7146 application memory-mapped I/O space (PCI region 0) occupied by the 626 board.

// Specific for 626

#define REV_A			TRUE		// Declare as a RevA model 626 board.
#define REV_B			FALSE		// Declare as a RevB, or higher, model 626 board.

#define SUBID_626		TRUE		// Attach to board only if PCI sub-IDs are valid.
#define SUBID_ANY		FALSE		// Attach to any board with an SAA7146 bridge.


///////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////   MODULE-LEVEL BOARD OBJECT   ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

typedef struct MOD_OBJ {				// ------------ MODULE-LEVEL BOARD OBJECT ------------

	// variables
	HBD			Board;			//   Logical board number.
	int			hSX;			//   Handle to kernel-mode driver.
	FPTR_ISR	IntCallback;		//   Address of application interrupt callback function.
	SX_CARD_REGISTER	CardReg;		//   Board attributes.
	DWORD			IntPriority;		//   Priority of interrupt thread.
	pthread_mutex_t		CriticalSection;	//   Synchronization structure for thread-safe execution.
	pthread_mutex_t		CriticalSectionAdc;	//   Synchronization structure for thread-safe ADC execution.
	HANDLE			hThreadEvent;		//   Sync object for main/interrupt threads.
	pthread_t		hIntThread;		//   Handle to interrupt thread.
	BOOLEAN			IsIntStopped;		//   Interrupt service has been terminated.
	SX_INTERRUPT		Intrpt;			//   WinDrvr interrupt control/status structure.
	DWORD			CallbackArg;		//   Interrupt callback argument.

	// pointers for Diagnostics functions:
	unsigned char		( *I2Cread )		( HBD, unsigned char );				// for S626_I2CRead().
	void			( *I2Cwrite )		( HBD, unsigned char, unsigned char );		// for S626_I2CWrite().
	unsigned short		( *DEBIread )		( HBD, unsigned short );			// for S626_RegRead().
	void			( *DEBIwrite )		( HBD, unsigned short, unsigned short );	// for S626_RegWrite().

	// pointers for Analog I/O functions:
	void			( *ResetADC )		( HBD, unsigned char * );			// for S626_ResetADC().
	void			( *ReadADC )		( HBD, unsigned short * );			// for S626_ReadADC().
	unsigned short		( *SingleADC )		( HBD, unsigned short );			// for S626_SingleADC().
	void			( *StartADC )		( HBD );					// for S626_StartADC().
	void			( *WaitDoneADC )	( HBD, unsigned short *);			// for S626_WaitDoneADC().
	void			( *WriteDAC )		( HBD, unsigned short, short );			// for S626_WriteDAC().
	void			( *WriteTrimDAC )	( HBD, unsigned char, unsigned char );		// for S626_WriteTrimDAC().

	// pointers for Battery functions:
	unsigned short		( *BackupEnableGet )	( HBD );					// for S626_BackupEnableGet().
	void			( *BackupEnableSet )	( HBD, unsigned short );			// for S626_BackupEnableSet().
	unsigned short		( *ChargeEnableGet )	( HBD );					// for S626_ChargeEnableGet().
	void			( *ChargeEnableSet )	( HBD, unsigned short );			// for S626_ChargeEnableSet().

	// pointers for Watchdog functions:
	unsigned short		( *WatchdogTimeout )	( HBD );					// for S626_WatchdogTimeout().
	unsigned short		( *WatchdogEnableGet )	( HBD );					// for S626_WatchdogEnableGet().
	void			( *WatchdogEnableSet )	( HBD, unsigned short );			// for S626_WatchdogEnableSet().
	unsigned short		( *WatchdogPeriodGet )	( HBD );					// for S626_WatchdogPeriodGet().
	void			( *WatchdogPeriodSet )	( HBD, unsigned short );			// for S626_WatchdogPeriodSet().
	void			( *WatchdogReset )	( HBD );					// for S626_WatchdogReset().

	// pointers for Digital I/O functions:
	unsigned short		( *DIOGroupRead )		( HBD, unsigned short );		// for S626_DIOReadBank().
	unsigned short		( *DIOGroupWriteGet )		( HBD, unsigned short );		// for S626_DIOWriteBankGet().
	void			( *DIOGroupWriteSet )		( HBD, unsigned short, unsigned short );// for S626_DIOWriteBankSet().
	unsigned short		( *DIOGroupEdgeGet )		( HBD, unsigned short );		// for S626_DIOEdgeGet().
	void			( *DIOGroupEdgeSet )		( HBD, unsigned short, unsigned short );// for S626_DIOEdgeSet().
	void			( *DIOGroupCapEnableSet )	( HBD, unsigned short, unsigned short, unsigned short );// for S626_DIOCapEnableSet().
	unsigned short		( *DIOGroupCapEnableGet )	( HBD, unsigned short );		// for S626_DIOCapEnableGet().
	unsigned short		( *DIOGroupCapStatus )		( HBD, unsigned short );		// for S626_DIOCapStatus().
	void			( *DIOGroupCapReset )		( HBD, unsigned short, unsigned short );// for S626_DIOCapReset().
	unsigned short		( *DIOGroupIntEnableGet )	( HBD, unsigned short );		// for S626_DIOIntEnableGet().
	void			( *DIOGroupIntEnableSet )	( HBD, unsigned short, unsigned short );// for S626_DIOIntEnableSet().
	unsigned short		( *DIOChanModeGet )		( HBD, unsigned short );		// for S626_DIOModeGet().
	void			( *DIOChanModeSet )		( HBD, unsigned short, unsigned short );// for S626_DIOModeSet().

	// pointers for Counter functions:
	DWORD   		( *CounterReadLatch )		( HBD, unsigned short );		// for S626_CounterReadLatch().
	void			( *CounterResetCapFlags )	( HBD, unsigned short );		// for S626_CounterCapFlagsReset().
	unsigned short		( *CounterCapStatus )		( HBD );				// for S626_CounterCapStatus().
	unsigned short		( *CounterModeGet )		( HBD, unsigned short );		// for S626_CounterModeGet().
	void			( *CounterModeSet )		( HBD, unsigned short, unsigned short );// for S626_CounterModeSet().
	void			( *CounterLatchSrcSet )		( HBD, unsigned short, unsigned short );// for S626_CounterLatchSourceSet().
	void			( *CounterEnableSet )		( HBD, unsigned short, unsigned short );// for S626_CounterEnableSet().
	void			( *CounterLoadTrigSet )		( HBD, unsigned short, unsigned short );// for S626_CounterLoadTrigSet().
	void			( *CounterSoftIndex )		( HBD, unsigned short );		// for S626_CounterSoftIndex().
        void			( *CounterPreload )		( HBD, unsigned short, U32 );	// for S626_CounterPreload().
	void			( *CounterIntSourceSet )	( HBD, unsigned short, unsigned short );// for S626_CounterIntSourceSet().

        U32	                ( *GetAddress )			( HBD );				// for S626_GetAddress().
        U32     		( *GetErrors )			( HBD );				// for S626_GetErrors().
	void			( *SetErrCallback )		( HBD, FPTR_ERR );			// for S626_SetErrCallback().
	void			( *InterruptStatus )		( HBD, unsigned short * );		// for S626_InterruptStatus().
	void			( *InterruptEnable )		( HBD, unsigned short );		// for S626_InterruptEnable().
} MOD_OBJ;


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  DATA STORAGE  ////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

static struct MOD_OBJ	mo[MAX_BOARDS];	//	= { {0} };	// Array of Model 626 board objects.
static UINT		open_boards = 0;			// Counts of opening boards


extern INT_ACTIONS S626CORE_IntActions;


//////////////////////////////////////////////////////////////////////////////////////////////////////
// Get release from automatically updated release string
static char *GetRelease( void )
{
	static char AutoRelease[] = "$Release:dev_1.0$";
	AutoRelease[sizeof(AutoRelease) - 2] = 0;
	return &AutoRelease[9];
}


///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////  FUNCTIONS used for in user Space  ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////  CRITICAL SECTION FUNCTIONS  ///////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Begin critical section.

void S626MOD_CriticalBegin( HBD bd )
{
	//EnterCriticalSection( &mo[bd].CriticalSection );
	pthread_mutex_lock( &mo[bd].CriticalSection );
}

///////////////////////////////////////////////////////////////////////////////
// End critical section.

void S626MOD_CriticalEnd( HBD bd )
{
	//LeaveCriticalSection( &mo[bd].CriticalSection );
	pthread_mutex_unlock( &mo[bd].CriticalSection );
}

///////////////////////////////////////////////////////////////////////////////
// Begin critical section.

void S626MOD_CriticalBeginAdc( HBD bd )
{
	//EnterCriticalSection( &mo[bd].CriticalSectionAdc );
	pthread_mutex_lock( &mo[bd].CriticalSectionAdc );
}

///////////////////////////////////////////////////////////////////////////////
// End critical section.

void S626MOD_CriticalEndAdc( HBD bd )
{
	//LeaveCriticalSection( &mo[bd].CriticalSectionAdc );
	pthread_mutex_unlock( &mo[bd].CriticalSectionAdc );
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////  MEMORY READ-WRITE FUNCTIONS  //////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Read a DWORD from a memory location.

DWORD S626MOD_ReadMem( PVOID address )
{
  ioc_param param;
  param.address = address;
  param.data = 0;
	//struct MOD_OBJ	*m = &mo[bd];
	struct MOD_OBJ	*m = &mo[0];	// let's always use first handle to execute reading,
					// since it doesn't matter to use which one and it makes core simpler.

	//printf("S626MOD_ReadMem: passsing in address %p \n", address);
	
	if ( ioctl( m->hSX, S626_IOC_ReadRegister, &param ) == 0 ) {
	  DWORD retval = param.data;
	  //printf("S626MOD_ReadMem: address %p, returned %x\n", address, retval);
	  return retval; 
	}
	else {
		return 0xffffffff;
	}

}

///////////////////////////////////////////////////////////////////////////////
// Write a DWORD to a memory location.

void S626MOD_WriteMem( PVOID address, DWORD data )
{
	ioc_param	prmt;
	//struct MOD_OBJ	*m = &mo[bd];
	struct MOD_OBJ	*m = &mo[0];	// let's always use first handle to execute writing,
					// since it doesn't matter to use which one and it makes core simpler.
	prmt.address =  address;
	prmt.data = data;
	//printf("S626MOD_WrMem: address %p, data %x\n", address,data);
	ioctl (m->hSX, S626_IOC_WriteRegister, &prmt);
}



///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////  INTERRUPT FUNCTIONS  /////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt thread entry point.  The interrupt handler executes on a private thread as it must
// suspend execution while waiting for WinDrvr to receive an interrupt request.  The thread entry
// point is implemented as a friend function because _beginthread() requires that any function
// launched on a new thread must be _cdecl, which is not compatible with class member functions.

static PVOID IntWait( PVOID pobj )
{
	// Cache pointer to the object.
	struct MOD_OBJ	*m = pobj;
	int		result;

	// Signal "interrupt thread active" to the main thread.
	//SetEvent( m->hThreadEvent );

	// Set the priority level of this interrupt thread.
	//SetThreadPriority( m->hIntThread, m->IntPriority );

	for ( ; ; )
	{
		// Suspend this thread until either (1) IRQ occurs, or (2) interrupt processing is terminated.
		// When we return from this function, the board's master IRQ will be disabled and the
		// interrupt request status flag will be negated.
		//SX_IntWait( m->hSX, &m->Intrpt );
		result = ioctl( m->hSX, S626_IOC_InterruptOn, &m->Intrpt );	//this call blocks until the interrupt happens

		// Log lost interrupts, if any.
		if ( m->Intrpt.dwLost )
			S626CORE_SetErrors( m->Board, ERR_LOST_IRQ );

		// Terminate interrupt thread if interrupt processing was halted by calling SX_IntDisable().
		if ( m->IsIntStopped || m->Intrpt.fStopped )
			break;

		// If we get to here, this must be a valid IRQ so we execute the application's ISR callback.
		//DA: 2/27/2007.  For 64 bit, we must have well defined callback functions.
		// Removing callback with no arguments.
		((FPTR_ISR) m->IntCallback)(m->CallbackArg);
	}
    return NULL;
}

////////////////////////////////////////////////////////////////////////////
// Set up interrupt service.

#define INT_THREAD_STACK		0x4000		// Interrupt thread stack size.

static void InterruptOpen( struct MOD_OBJ *m )
{
	int ret;

	// Indicate that interrupt services have not been terminated by InterruptClose().
	m->IsIntStopped = FALSE;

	// Attempt to enable kernel-mode interrupt.
	//if ( !SX_IntEnable( m->hSX, &m->Intrpt ) )
	if ( ioctl( m->hSX, S626_IOC_RequestIrq, &m->Intrpt ) )
	{
		S626CORE_SetErrors( m->Board, ERR_INTERRUPT );
		return;
	}

	// Abort if kernel-mode interrupt could not be allocated.
	if ( m->Intrpt.dwInterruptNum < 1 )
	{
		S626CORE_SetErrors( m->Board, ERR_INTERRUPT );
		return;
	}

	// Create an event to synchronize startup of the interrupt thread.
	//m->hThreadEvent = CreateEvent( 0, TRUE, FALSE, 0 );

	// Attempt to launch the interrupt thread for this board object.
	//m->hIntThread = (HANDLE)_beginthread( IntWait, INT_THREAD_STACK, (PVOID)m );
	m->hIntThread = 0;
	ret = pthread_create( &m->hIntThread, NULL, IntWait, (PVOID)m );

	// If the thread launched properly, wait for the thread to activate, otherwise set error flag.
	//if ( m->hIntThread != (HANDLE)0xFFFFFFFF )			// If thread launched properly,
	//	WaitForSingleObject( m->hThreadEvent, INFINITE );	//   wait for the thread to activate.
	//else								// Otherwise,
	if ( ret != 0 )			// If thread not launched properly,
		S626CORE_SetErrors( m->Board, ERR_THREAD );		//   set error flag.

	// Destroy the thread syncronization event.
	//CloseHandle( m->hThreadEvent );
}

///////////////////////////////////////////////////////////////////////////////
// Terminate interrupt thread.

static void InterruptClose( struct MOD_OBJ *m )
{
	int result;

	// Set a flag to let the interrupt thread know that interrupt services
	// have been terminated.  This is necessary and is for simulating WinDriver 
	// since the Windriver fStopped flag does not seem to be set properly.
	m->IsIntStopped = TRUE;
	
	// Force the interrupt handler to return from its SX_IntWait() call.
	//SX_IntDisable( m->hSX, &m->Intrpt );
	result = ioctl( m->hSX, S626_IOC_InterruptOff, &m->Intrpt );

    // Wait for the interrupt thread to terminate before continuing.
	//WaitForSingleObject( m->hIntThread, INFINITE );
	pthread_join( m->hIntThread, NULL );

}

//////////////////////////////////////////////////////////////////////////////////////////////
// Translate a list of action items into kernel-mode driver equivalents.
/*
static void TransformActions( HBD bd, SX_TRANSFER *pXfer, ACTION_LIST *pActList, DWORD RegXferBase )
{
	UINT		i;
	ACCESS_SPEC	*pAct = pActList->Actions;

	for ( i = 0; i < pActList->NumActions; i++, pXfer++, pAct++ )
	{
		pXfer->cmdTrans		= pAct->IsWrite ? WM_DWORD : RM_DWORD;		// Op: DWORD read or write.
		pXfer->dwPort		= pAct->AdrsOS + RegXferBase;			// Address of target reg.
		pXfer->Data.Dword	= pAct->DataVal;				// Data to write to reg.
		pXfer->dwOptions	= 0;
	}
}
*/


///////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////  INIT & SHUTDOWN FUNCTIONS  /////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Attempt to validate, lock and register a Model 626 board.  If requested, the PCI SubDeviceID and
// SubVendorID values are validated, then an attempt is made to lock and register the board.  If the
// board is successfully validated and locked, the base class values MyBase and MyAddress are set
// accordingly and the board is opened.
//
// Imports:
//	Loc			= address of SX_PCI_SLOT structure for target board.
//	MatchSubIDs	= boolean request to validate PCI SubVendorID and SubDeviceID.
//	IsBoardRevA	= boolean that specifies target board is to be treated as a RevA model 626.

static void LockBoard( struct MOD_OBJ *m, SX_PCI_SLOT *Loc, BOOLEAN MatchSubIDs, BYTE IsBoardRevA )
{
	SX_PCI_CARD_INFO	PciCardInfo;		// Attributes of the specified PCI card.
	SX_PCI_CONFIG_DUMP	CfgDump;		// Structure used to control PCI board configuration dumps.
	WORD			SubVendorID;
	WORD			SubDeviceID;
	UINT			i;
	register SX_ITEMS	*pItemMemory;		// Ptr to memory item, which is the window into 626 reg space.
	register SX_ITEMS	*pItemInterrupt;	// Ptr to interrupt item.
	SX_ITEMS		*pItem;

	// Indicate that we have not yet encountered any errors.
	S626CORE_SetErrors( m->Board, 0 );

	// Get card info (interrupts, I/O, memory), and indicate that the board has not
	// been registered with the kernel-mode driver.
	PciCardInfo.pciSlot = *Loc;
	//SX_PciGetCardInfo( m->hSX, &PciCardInfo );
	if ( ioctl( m->hSX, S626_IOC_GetCardInfo, &PciCardInfo ) )
	{
		S626CORE_SetErrors( m->Board, ERR_CARDREG );	//ErrFlags |= ERR_CARDREG;
		return;
	}
	m->CardReg.Card = PciCardInfo.Card;
	m->CardReg.hCard = 0;
	// Determine if card has the correct PCI SubVendorID and SubDeviceID.
	if ( MatchSubIDs )
	{/*
		// Copy SubVendorID and SubDeviceID from board's PCI configuration registers to SubBuffer.
		CfgDump.pciSlot  = *Loc;				// Specify dwBus, dwSlot, dwFunction.
		CfgDump.fIsRead  = TRUE;				// Specify config space read operation.
		CfgDump.dwOffset = 0;					// Adrs offset of data in config space.
		SX_PciConfigDump( m->hSX, &CfgDump );			// Copy board's SubIDs to buffer.
		SubVendorID = CfgDump.pciConfig.u.type0.SubVendorID;
		SubDeviceID = CfgDump.pciConfig.u.type0.SubSystemID;

		// Exit if configuration dump was unsuccessful.
		if ( CfgDump.dwResult < 2 )
		{
			S626CORE_SetErrors( m->Board, ERR_CFGDUMP | ERR_CARDREG );
			return;
		}

		// Exit if SubIDs don't match.
		if ( ( SubDeviceID != SUBDEVICEID626 ) || ( SubVendorID != SUBVENDORID626 ) )
		{
			S626CORE_SetErrors( m->Board, ERR_SUBIDS | ERR_CARDREG );
			return;
		}*/
	  // Not implemented yet for Linux users
	}

	// Attempt to lock and register card with kernel-mode driver.
	m->CardReg.fCheckLockOnly = FALSE;
	for ( i = 0; i < m->CardReg.Card.dwItems; i++ )
		m->CardReg.Card.Item[i].fNotSharable = FALSE;
	//SX_CardRegister( m->hSX, &m->CardReg );
	if (ioctl (m->hSX, S626_IOC_Register, &m->CardReg))
	{
		S626CORE_SetErrors( m->Board, ERR_CARDREG );	//ErrFlags |= ERR_CARDREG;
		return;
	}

	// Exit if card could not be locked and registered.
	if ( !m->CardReg.hCard )
	{
		S626CORE_SetErrors( m->Board, ERR_CARDREG );
		return;
	}

	// Get pointers to card's memory and interrupt items.
	pItem = &( m->CardReg.Card.Item[0] );
	for ( i = 0; i < m->CardReg.Card.dwItems; i++, pItem++ )
	{
		switch ( pItem->item )
		{
		case ITEM_MEMORY:		pItemMemory    = pItem;	break;
		case ITEM_INTERRUPT:	pItemInterrupt = pItem;	break;
		}
	}

        // printf("LockBoard Set Base %p\n", pItemMemory->I.Mem.pUserDirectAddr);
	// Cache attributes of card's memory item.
	S626CORE_SetBase( m->Board, pItemMemory->I.Mem.pUserDirectAddr );		// Direct access base address. AKA mmio BAR
    
	// Save bus number and slot number of card.
	S626CORE_SetAddress( m->Board, ( Loc->dwBus << 16 ) | Loc->dwSlot );

	// Init board class and hardware state, allocate DMA buffers, etc.
	S626CORE_BoardOpen( m->Board, IsBoardRevA );
	//printf("board open done Set Base %p\n", pItemMemory->I.Mem.pUserDirectAddr);
	// If app will be using interrupts, set up the interrupt structures and launch the interrupt thread.
	if ( m->IntCallback )
	{
		// Set up kernel-mode driver's IRQ status check mask.  When an interrupt occurs, the kernel-mode
		// driver reads the IRQ status register (by executing the operations specified in StatusActions)
		// and masks the IRQ bit to determine whether this board is requesting service.  If the board is
		// requesting service, the kernel-mode driver executes the operations specified in ProcessActions
		// to negate the board's IRQ request.  This scheme is required because some PCI systems share the
		// target board's IRQ.
		m->Intrpt.dwAndMask	= S626CORE_IntActions.StatusAndMask;		// IRQ status AND mask.
		m->Intrpt.dwXorMask	= S626CORE_IntActions.StatusXorMask;		// IRQ status XOR mask.
/* ?
		// Set up ISR transfer cmds by translating action items into kernel-mode driver equivalents.
		// Translate and store the actions that return IRQ status.
		m->Intrpt.dwStatusCmds = S626CORE_IntActions.StatusActions.NumActions;
		TransformActions( m->Board, m->Intrpt.StatusCmd, &S626CORE_IntActions.StatusActions,  pItemMemory->I.Mem.dwTransAddr );
		// Translate and store the actions that negate the board's IRQ.
		m->Intrpt.dwProcessCmds = S626CORE_IntActions.ProcessActions.NumActions;
		TransformActions( m->Board, m->Intrpt.ProcessCmd, &S626CORE_IntActions.ProcessActions, pItemMemory->I.Mem.dwTransAddr );
*/
		// Set up remainder of interrupt control structure.
		m->Intrpt.hInterrupt		= pItemInterrupt->I.Int.hInterrupt;	// Driver's interrupt handle.
		m->Intrpt.dwInterruptNum	= pItemInterrupt->I.Int.dwInterrupt;	// IRQ to use.
		m->Intrpt.fNotSharable		= FALSE;				// Interrupt is shareable.
		m->Intrpt.dwOptions			= INTERRUPT_LEVEL_SENSITIVE;	// IRQ is level sensitive.
		m->Intrpt.fStopped			= FALSE;			// Not yet halted by SX_IntDisable().

		// Launch the interrupt thread.
		InterruptOpen( m );
	}
}

//////////////////////////////////////////////////////////////////////////////
// Register 626 board at the specified location.

static void RegisterSpecific( struct MOD_OBJ *m, BYTE FiltAdrs, BYTE IsBoardRevA, DWORD PhysLoc )
{
	SX_PCI_SLOT Loc;

	// Init structure to indicate board's physical location.
	Loc.dwBus	= PhysLoc >> 16;
	Loc.dwSlot	= PhysLoc & 0xFFFFL;
	Loc.dwFunction	= 0;

	// Attempt to lock and register the board.
	LockBoard( m, &Loc, FiltAdrs, IsBoardRevA );
}

/////////////////////////////////////////////////////////////////////////////////////
// Register specified available 626 board at any location. If MatchSubIDs is asserted,
// use PCI sub device and vendor IDs to qualify board.

static void RegisterAny( struct MOD_OBJ *m, BOOLEAN MatchSubIDs, BYTE IsBoardRevA )
{
	UINT c;

	// Enumerate all PCI cards that have matching 626 VendorId and DeviceID.
	SX_PCI_SCAN_CARDS PciScan;		// List of 626 cards.
	PciScan.searchId.dwVendorId = VENDORID626;
	PciScan.searchId.dwDeviceId = DEVICEID626;

	//SX_PciScanCards( m->hSX, &PciScan );
	if (ioctl (m->hSX, S626_IOC_Init, &PciScan))
	{
		S626CORE_SetErrors( m->Board, ERR_CARDREG );	// ErrFlags |= ERR_CARDREG;
		return;
	}

	// If at least one matching card was found, register the board that is specified and not locked.
	for ( c = 0; c < PciScan.dwCards; c++ )
	{
	    if ( c == m->Board )
	    {
		// Attempt to lock and register this board.
		//LockBoard( m, &PciScan.cardSlot[c], MatchSubIDs, IsBoardRevA );
		LockBoard( m, &PciScan.cardSlot[m->Board], MatchSubIDs, IsBoardRevA );

		// Exit if board was successfully registered.
		if ( m->CardReg.hCard )
			return;
	    }
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Module-level object constructors.
// Imports:
//	PhysLoc      = PCI BIOS reference to board's physical location: (BusNum << 16) | SlotNum.
//	Callback     = Address of application interrupt callback.  If application will not be
//				    using interrupts, set to zero.
//	Priority     = Windows priority level for interrupt thread.
//  callback_arg = Constant value to pass as interrupt callback argument.  Set to zero
//                  if callback function is not to be passed an argument.

static void Constructor( HBD bd, DWORD PhysLoc, FPTR_ISR callback_fn, DWORD Priority, DWORD callback_arg )
{
	struct MOD_OBJ		*m = &mo[bd];
	pthread_mutexattr_t	attributes;

	m->Board = bd;
	// Initialize private interrupt parameters.
	m->Intrpt.dwLost	= 0;		// Indicate no lost interrupts detected.
	m->CardReg.hCard	= 0;		// Indicate this card is not registered.
	m->IntCallback		= callback_fn;	// Init pointer to interrupt callback function.
	m->CallbackArg		= callback_arg;	// Save the interrupt callback argument.

	// Initialize board object's Vfunc pointers.
				// pointers for Diagnostics functions:
	m->I2Cread		= S626CORE_I2Cread;			// U8 S626CORE_I2Cread( HBD bd, U8 addr )
	m->I2Cwrite		= S626CORE_I2Cwrite;			// void S626CORE_I2Cwrite( HBD bd, U8 addr, U8 val )
	m->DEBIread		= S626CORE_DEBIread;			// U16 S626CORE_DEBIread( HBD bd, U16 addr )
	m->DEBIwrite		= S626CORE_DEBIwrite;			// void S626CORE_DEBIwrite( HBD bd, U16 addr, U16 wdata )
				// pointers for Analog I/O functions:
	m->ResetADC		= S626CORE_ResetADC;			// void S626CORE_ResetADC( HBD bd, U8 *ppl )
	m->ReadADC		= S626CORE_ReadADC;			// void S626CORE_ReadADC( HBD bd, U16 *pdata )
	m->SingleADC		= S626CORE_SingleADC;			// U16 S626CORE_SingleADC( HBD bd, U16 AdcSpec )
	m->StartADC		= S626CORE_StartADC;			// void S626CORE_StartADC( HBD bd )
	m->WaitDoneADC		= S626CORE_WaitDoneADC;			// void S626CORE_WaitDoneADC( HBD bd, U16 *pdata )
	m->WriteDAC		= S626CORE_WriteDAC;			// void S626CORE_WriteDAC( HBD bd, U16 chan, S16 dacdata )
	m->WriteTrimDAC		= S626CORE_WriteTrimDAC;		// void S626CORE_WriteTrimDAC( HBD bd, U8 logchan, U8 val )
				// pointers for Battery functions:
	m->BackupEnableGet	= S626CORE_BackupEnableGet;		// U16 S626CORE_BackupEnableGet( HBD bd )
	m->BackupEnableSet	= S626CORE_BackupEnableSet;		// void S626CORE_BackupEnableSet( HBD bd, U16 enab )
	m->ChargeEnableGet	= S626CORE_ChargeEnableGet;		// U16 S626CORE_ChargeEnableGet( HBD bd )
	m->ChargeEnableSet	= S626CORE_ChargeEnableSet;		// void S626CORE_ChargeEnableSet( HBD bd, U16 enab )
				// pointers for Watchdog functions:
	m->WatchdogTimeout	= S626CORE_WatchdogTimeout;		// U16 S626CORE_WatchdogTimeout( HBD bd )
	m->WatchdogEnableGet	= S626CORE_WatchdogEnableGet;		// U16 S626CORE_WatchdogEnableGet( HBD bd )
	m->WatchdogEnableSet	= S626CORE_WatchdogEnableSet;		// void S626CORE_WatchdogEnableSet( HBD bd, U16 enab )
	m->WatchdogPeriodGet	= S626CORE_WatchdogPeriodGet;		// U16 S626CORE_WatchdogPeriodGet( HBD bd )
	m->WatchdogPeriodSet	= S626CORE_WatchdogPeriodSet;		// void S626CORE_WatchdogPeriodSet( HBD bd, U16 value )
	m->WatchdogReset	= S626CORE_WatchdogReset;		// void S626CORE_WatchdogReset( HBD bd )
				// pointers for Digital I/O functions:
	m->DIOGroupRead		= S626CORE_DIOGroupRead;		// U16 S626CORE_DIOGroupRead( HBD bd, U16 group )
	m->DIOGroupWriteGet	= S626CORE_DIOGroupWriteGet;		// U16 S626CORE_DIOGroupWriteGet( HBD bd, U16 group )
	m->DIOGroupWriteSet	= S626CORE_DIOGroupWriteSet;		// void S626CORE_DIOGroupWriteSet( HBD bd, U16 group, U16 value )
	m->DIOGroupEdgeGet	= S626CORE_DIOGroupEdgeGet;		// U16 S626CORE_DIOGroupEdgeGet( HBD bd, U16 group )
	m->DIOGroupEdgeSet	= S626CORE_DIOGroupEdgeSet;		// void S626CORE_DIOGroupEdgeSet( HBD bd, U16 group, U16 value )
	m->DIOGroupCapEnableSet	= S626CORE_DIOGroupCapEnableSet;	// void S626CORE_DIOGroupCapEnableSet( HBD bd, U16 group, U16 value, U16 enab )
	m->DIOGroupCapEnableGet	= S626CORE_DIOGroupCapEnableGet;	// U16 S626CORE_DIOGroupCapEnableGet( HBD bd, U16 group )
	m->DIOGroupCapStatus	= S626CORE_DIOGroupCapStatus;		// U16 S626CORE_DIOGroupCapStatus( HBD bd, U16 group )
	m->DIOGroupCapReset	= S626CORE_DIOGroupCapReset;		// void S626CORE_DIOGroupCapReset( HBD bd, U16 group, U16 value )
	m->DIOGroupIntEnableGet	= S626CORE_DIOGroupIntEnableGet;	// U16 S626CORE_DIOGroupIntEnableGet( HBD bd, U16 group)
	m->DIOGroupIntEnableSet	= S626CORE_DIOGroupIntEnableSet;	// void S626CORE_DIOGroupIntEnableSet( HBD bd, U16 group, U16 value)
	m->DIOChanModeGet	= S626CORE_DIOChanModeGet;		// U16 S626CORE_DIOChanModeGet( HBD bd, U16 DIOnum )
	m->DIOChanModeSet	= S626CORE_DIOChanModeSet;		// void S626CORE_DIOChanModeSet( HBD bd, U16 DIOnum, U16 value )
				// pointers for Counter functions:
	m->CounterReadLatch	= S626CORE_CounterReadLatch;		// U32 S626CORE_CounterReadLatch( HBD bd, U16 chan )
	m->CounterResetCapFlags	= S626CORE_CounterResetCapFlags;	// void S626CORE_CounterResetCapFlags( HBD bd, U16 chan )
	m->CounterCapStatus	= S626CORE_CounterCapStatus;		// U16 S626CORE_CounterCapStatus( HBD bd )
	m->CounterModeGet	= S626CORE_CounterModeGet;		// U16 S626CORE_CounterModeGet( HBD bd, U16 chan )
	m->CounterModeSet	= S626CORE_CounterModeSet;		// void S626CORE_CounterModeSet( HBD bd, U16 chan, U16 setup )
	m->CounterLatchSrcSet	= S626CORE_CounterLatchSrcSet;		// void S626CORE_CounterLatchSrcSet( HBD bd, U16 chan, U16 value )
	m->CounterEnableSet	= S626CORE_CounterEnableSet;		// void S626CORE_CounterEnableSet( HBD bd, U16 chan, U16 enab )
	m->CounterLoadTrigSet	= S626CORE_CounterLoadTrigSet;		// void S626CORE_CounterLoadTrigSet( HBD bd, U16 chan, U16 Trig )
	m->CounterSoftIndex	= S626CORE_CounterSoftIndex;		// void S626CORE_CounterSoftIndex( HBD bd, U16 chan )
	m->CounterPreload	= S626CORE_CounterPreload;		// void S626CORE_CounterPreload( HBD bd, U16 chan, U32 value )
	m->CounterIntSourceSet	= S626CORE_CounterIntSourceSet;		// void S626CORE_CounterIntSourceSet( HBD bd, U16 chan, U16 IntSource )
				// pointers for Status and control functions:
	m->GetAddress		= S626CORE_GetAddress;			// U32 S626CORE_GetAddress( HBD bd )
	m->GetErrors		= S626CORE_GetErrors;			// U32 S626CORE_GetErrors( HBD bd )
	m->SetErrCallback	= S626CORE_SetErrCallback;		// void S626CORE_SetErrCallback( HBD bd, FPTR_ERR Callback )
	m->InterruptStatus	= S626CORE_InterruptStatus;		// void S626CORE_InterruptStatus( HBD bd, U16 *IntFlags )
	m->InterruptEnable	= S626CORE_InterruptEnableSet;		// void S626CORE_InterruptEnableSet( HBD bd, U16 enab )

	// Take priority level of private interrupt thread.
	m->IntPriority = Priority;
	// Attempt to open a private kernel-mode driver, exit if attempt fails.
	if ( m->hSX == 0 )
	{
		switch ( m->Board )			// valid:  Board(bd) = 0, 1, 2, 3
		{
			case 0:
					m->hSX = open ("/dev/s626a0", O_RDWR);	// for 1st board
					if ( m->hSX<=0 ) 
					{
						printf("Board(%d) - \"/dev/s626a0\" can not be opened.\n", bd);
						S626CORE_SetErrors( m->Board, ERR_OPEN );	// ErrFlags |= ERR_OPEN
						return;
					}
					printf("Board(%d) - \"/dev/s626a0\" is opened.\n", bd);
					break;
			case 1:
					m->hSX = open ("/dev/s626a1", O_RDWR);	// for 2nd board
					if ( m->hSX<=0 ) 
					{
						printf("Board(%d) - \"/dev/s626a1\" can not be opened.\n", bd);
						S626CORE_SetErrors( m->Board, ERR_OPEN );	// ErrFlags |= ERR_OPEN
						return;
					}
					printf("Board(%d) - \"/dev/s626a1\" is opened.\n", bd);
					break;
			case 2:
					m->hSX = open ("/dev/s626a2", O_RDWR);	// for 3rd board
					if ( m->hSX<=0 ) 
					{
						printf("Board(%d) - \"/dev/s626a2\" can not be opened.\n", bd);
						S626CORE_SetErrors( m->Board, ERR_OPEN );	// ErrFlags |= ERR_OPEN
						return;
					}
					printf("Board(%d) - \"/dev/s626a2\" is opened.\n", bd);
					break;
			case 3:
					m->hSX = open ("/dev/s626a3", O_RDWR);	// for 4th board
					if ( m->hSX<=0 ) 
					{
						printf("Board(%d) - \"/dev/s626a3\" can not be opened.\n", bd);
						S626CORE_SetErrors( m->Board, ERR_OPEN );	// ErrFlags |= ERR_OPEN
						return;
					}
					printf("Board(%d) - \"/dev/s626a3\" is opened.\n", bd);
					break;
			default:
					printf("Invalid board number - Board(%d). So, it can not be opened.\n", bd);
					return;
		}
	} else {
		printf("Board(%d) is already been opened.\n", bd);
		return;
	}

	// Initialize the critical sections before the interrupt thread is launched.
	//InitializeCriticalSection( &m->CriticalSection );
	//InitializeCriticalSection( &m->CriticalSectionAdc );
	// Assume using default Mutex Attributes for CriticalSection & CriticalSectionAdc:
	//			pshared				PTHREAD_PROCESS_PRIVATE 
	//			kind (non portable)		PTHREAD_MUTEX_NONRECURSIVE_NP 
	//			name (non portable)		PTHREAD_DEFAULT_MUTEX_NAME_NP 
	//			type				PTHREAD_MUTEX_DEFAULT (PTHREAD_MUTEX_NORMAL)
	// So, it is not needed to execute following:
	//
	//  pthread_mutexattr_t       attributes;
	//
	//  if (!pthread_mutexattr_init (&attributes))
	//  {
	//		pthread_mutex_init (&MyCriticalSection, &attributes);
	//		pthread_mutexattr_destroy (&attributes);
	//  }

	// Register the 626 board with the kernel-mode driver.  There are various options for registering the
	// different board flavors:
	//    * RevA has its serial EEPROM mapped to the "old" (original) address.  This was implemented with a
	//		Fairchild (or equivalent) EEPROM with no modifications.  PCI subIDs are not supported.  These boards
	//		are used only by early 626 customers; newer customers can't use these boards.
	//    * RevB has its EEPROM mapped to the "new" address, which allows PCI subIDs to be employed.  This was
	//		implemented with a Fairchild (or equivalent) EEPROM with pins 2 and 3 shorted together.  These boards
	//		may be used by all 626 customers except the very early 626 customers.
	//    * RevAB uses a Microchip (or equivalent) EEPROM, which is simultaneously mapped to the old and new
	//		addresses.  It doesn't matter whether pins 2 and 3 are shorted together because the device responds
	//		at all addresses.  PCI subIDs are supported.  These boards may be used by any 626 customer, new or old.
	switch ( PhysLoc )
	{
	case 0xFFFFFFFC:
		// Register RevA or RevAB board with any subIDs ---
		// This option is for Sensoray internal use only; it is not intended for use by customers.
		// The physical location of the 626 board is not specified, so we will try to register the first
		// available board, declared as RevA, that has any PCI sub-IDs.  If no such board is found,
		// the registration will fail.  This option is used for accessing RevA or RevAB boards that have
		// any PCI sub-IDs.  If the registration fails, the board is either RevB, or it is a RevA/RevAB and
		// the EEPROM has malfunctioned.
		RegisterAny( m, SUBID_ANY, REV_A );
		break;

	case 0xFFFFFFFD:
		// Register RevA or RevAB board with valid subIDs ---
		// This option is for Sensoray internal use only; it is not intended for use by customers.
		// The physical location of the 626 board is not specified, so we will try to register the first
		// available board, declared as RevA, that has valid PCI sub-IDs.  If no such board is found,
		// the registration will fail.  This option is used for accessing RevA or RevAB boards that have
		// valid PCI sub-IDs.  If the registration fails, the board is either RevB, or it is a RevA and
		// the EEPROM has malfunctioned or has not yet had the PCI sub-IDs programmed into it.
		RegisterAny( m, SUBID_626, REV_A );
		break;

	case 0xFFFFFFFE:
		// Register RevB or RevAB board with valid subIDs ---
		// This option is for Sensoray internal use only; it is not intended for use by customers.
		// The physical location of the 626 board is not specified, so we will try to register the first
		// available board, declared as RevB, that has valid PCI sub-IDs.  If no such board is found,
		// the registration will fail.  This option is used for accessing ONLY RevB/RevAB boards that have
		// valid PCI sub-IDs.  If the registration fails, the board is either RevA, or it is a RevB/RevAB and
		// the EEPROM has malfunctioned or has not yet had the PCI sub-IDs programmed into it.
		RegisterAny( m, SUBID_626, REV_B );
		break;

	case 0xFFFFFFFF:
		// Register RevB or RevAB board with any subIDs ---
		// This option is for Sensoray internal use only; it is not intended for use by customers.
		// The physical location of the 626 board is not specified, so we will try to register the first
		// available board, declared as RevB, that has an SAA7146 bridge device.  If no such board is
		// found, the registration will fail.  This option is used for registering RevB/RevAB boards before
		// the EEPROM has been programmed with valid sub-IDs.  If a RevA board has been registered, this
		// can be detected by attempting to access the EEPROM; the EEPROM will be accessible if the board
		// is RevB, but not if it is a RevA.
		RegisterAny( m, SUBID_ANY, REV_B );
		break;

	case 0x00000000:
		// The physical location of the 626 board is not specified, so we will try to register the
		// first available board, declared as RevB, that has valid PCI sub-IDs.
		RegisterAny( m, SUBID_626, REV_B );
		// If no matching boards were found, we will now try to register the first available board,
		// declared as RevA, that has an SAA7146 bridge device (sub-IDs are ignored).
		if ( !m->CardReg.hCard )
			RegisterAny( m, SUBID_ANY, REV_A );
		break;

	default:
		// The physical location of the 626 board has been explicitly specified, so we will try to
		// register the board at the specified location by first declaring it to be a non-RevA board
		// with valid sub-IDs.
		RegisterSpecific( m, SUBID_626, REV_B, PhysLoc );
		// If the registration attempt failed, we will again try to register the board by declaring it to
		// be a RevA board and ignoring the board's sub-IDs.
		if ( !m->CardReg.hCard )
			RegisterSpecific( m, SUBID_ANY, REV_A, PhysLoc );
	}

	// Make sure error flag is set if card is not registered.
	if ( !m->CardReg.hCard )
		S626CORE_SetErrors( m->Board, ERR_CARDREG );	// ErrFlags |= ERR_CARDREG;
}

//////////////////////////////////////////////////////////////////////////
// Interface class destructor.

static void Destructor( HBD bd )
{
	struct MOD_OBJ	*m = &mo[bd];
	int		result;

	// If kernel-mode driver is open ...
	if ( m->hSX )
	{
		// If Model 626 board is registered with kernel-mode driver ...
		if ( m->CardReg.hCard )
		{
			// If application is using interrupts, shut down interrupts.
			if ( m->IntCallback ) {
				InterruptClose( m );
			}

			// Shut down board hardware, release DMA buffers, etc.
			S626CORE_BoardClose( m->Board );

			// Unregister card with kernel-mode driver.
			//SX_CardUnregister( m->hSX, &m->CardReg );
			result = ioctl( m->hSX, S626_IOC_Unregister, &m->CardReg );
		}

		// Delete the critical sections.
		//DeleteCriticalSection( &m->CriticalSection );
		//DeleteCriticalSection( &m->CriticalSectionAdc );
		pthread_mutex_destroy( &m->CriticalSection );
		pthread_mutex_destroy( &m->CriticalSectionAdc );

		// Close the kernel-mode driver.
		//SX_Close( m->hSX );
		if ( close(m->hSX) ) printf ("Unable to close driver.\n"); else m->hSX = 0;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////  DMA BUFFER MANAGEMENT FUNCTIONS  ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Allocate a DMA buffer that begins on a physical page boundary address.

void S626MOD_AllocDMAB( HBD bd, DWORD nBytes, DMABUF *BdDmaBuf )
{
	struct MOD_OBJ *m = &mo[bd];
	//register SX_DMA *pwddma;
	ioc_param 	dma_arg;

	// Indicate to board class that DMA buffer is not yet allocated.
	BdDmaBuf->DMAHandle = 0;

	// Allocate memory for the kernel-mode DMA management structure.
	//pwddma = (SX_DMA *) malloc( sizeof(SX_DMA) );
	dma_arg.data = nBytes;
	dma_arg.address = m->CardReg.hCard;
	if ( ioctl( m->hSX, S626_IOC_AllocDMAB, &dma_arg ) )
	{
		S626CORE_SetErrors( bd, ERR_ALLOC_MEMORY );
		return;
	}

	// Return the kernel-mode driver's DMA buffer handle and the page 1 physical/logical base
	// addresses to the Model 626 base class.
	BdDmaBuf->PhysicalBase	= dma_arg.PhysicalBase;
	BdDmaBuf->LogicalBase	= dma_arg.LogicalBase;
	BdDmaBuf->DMAHandle	= dma_arg.DMAHandle;

}

///////////////////////////////////////////////////////////////////////////
// Release DMA buffer.

void S626MOD_CloseDMAB( HBD bd, DMABUF *pDmaBuf )
{
	struct MOD_OBJ	*m = &mo[bd];
	ioc_param 	dma_arg;

	// Get handle to kernel-mode DMA management structure.
	//SX_DMA *pwddma = (SX_DMA *) pDmaBuf->DMAHandle;

	dma_arg.PhysicalBase = pDmaBuf->PhysicalBase;
	dma_arg.LogicalBase = pDmaBuf->LogicalBase;
	dma_arg.DMAHandle = pDmaBuf->DMAHandle;
	dma_arg.address =  m->CardReg.hCard;
	if ( ioctl( m->hSX, S626_IOC_CloseDMAB, &dma_arg ) )
		S626CORE_SetErrors( bd, ERR_ALLOC_MEMORY );

}



// Function macros -------------------------
//#define BD					p626[hbd]			// Shorthand for pointer to current board object.
#define	BD					mo[hbd]				// Shorthand for pointer to current board object.

// Local wide-scope variables --------------
//static Cls626					*p626[MAX_NUM_BOARDS];		// Array of pointers to Model 626 board objects.
//static struct MOD_OBJ			*pmo626[MAX_NUM_BOARDS];


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////  API (Application Programming Interface) FUNCTIONS  /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Diagnostics:
unsigned char	S626_I2CRead			(HBD hbd, unsigned char addr)				{ return BD.I2Cread(hbd, addr); }
void		S626_I2CWrite			(HBD hbd, unsigned char addr, unsigned char data)	{ BD.I2Cwrite(hbd, addr, data); }
unsigned short  S626_RegRead			(HBD hbd, unsigned short addr)				{ return BD.DEBIread(hbd, addr); }
void		S626_RegWrite			(HBD hbd, unsigned short addr, unsigned short data)	{ BD.DEBIwrite(hbd, addr, data); }

// Analog I/O functions:
void		S626_ResetADC			(HBD hbd, unsigned char *ppl)			{ BD.ResetADC(hbd, ppl); }
void		S626_ReadADC			(HBD hbd, unsigned short *pdata)		{ BD.ReadADC(hbd, pdata); }
unsigned short	S626_SingleADC			(HBD hbd, unsigned short spec)			{ return BD.SingleADC(hbd, spec); }
void		S626_StartADC			(HBD hbd)					{ BD.StartADC(hbd); }
void		S626_WaitDoneADC		(HBD hbd, unsigned short *pdata)		{ BD.WaitDoneADC(hbd, pdata ); }
void		S626_WriteDAC			(HBD hbd, unsigned short ch, long val)		{ BD.WriteDAC(hbd, ch, (short)(val & 0xFFFF)); }
void		S626_WriteTrimDAC		(HBD hbd, unsigned char ch, unsigned char val)	{ BD.WriteTrimDAC(hbd, ch, val); }

// Battery functions:
unsigned short  S626_BackupEnableGet		(HBD hbd)					{ return BD.BackupEnableGet(hbd); }
void		S626_BackupEnableSet		(HBD hbd, unsigned short en)			{ BD.BackupEnableSet(hbd, en); }
unsigned short  S626_ChargeEnableGet		(HBD hbd)					{ return BD.ChargeEnableGet(hbd); }
void		S626_ChargeEnableSet		(HBD hbd, unsigned short en)			{ BD.ChargeEnableSet(hbd, en); }

// Watchdog functions:
unsigned short  S626_WatchdogTimeout		(HBD hbd)					{ return BD.WatchdogTimeout(hbd); }
unsigned short  S626_WatchdogEnableGet		(HBD hbd)					{ return BD.WatchdogEnableGet(hbd); }
void		S626_WatchdogEnableSet		(HBD hbd, unsigned short en)			{ BD.WatchdogEnableSet(hbd, en); }
unsigned short  S626_WatchdogPeriodGet		(HBD hbd)					{ return BD.WatchdogPeriodGet(hbd); }
void		S626_WatchdogPeriodSet		(HBD hbd, unsigned short val)			{ BD.WatchdogPeriodSet(hbd, val); }
void		S626_WatchdogReset		(HBD hbd)					{ BD.WatchdogReset(hbd); }

// Digital I/O functions:
unsigned short  S626_DIOReadBank		(HBD hbd, unsigned short grp)				{ return BD.DIOGroupRead(hbd, grp); }
unsigned short  S626_DIOWriteBankGet		(HBD hbd, unsigned short grp)				{ return BD.DIOGroupWriteGet(hbd, grp); }
void		S626_DIOWriteBankSet		(HBD hbd, unsigned short grp, unsigned short val)	{ BD.DIOGroupWriteSet(hbd, grp, val); }
unsigned short  S626_DIOEdgeGet			(HBD hbd, unsigned short grp)				{ return BD.DIOGroupEdgeGet(hbd, grp); }
void		S626_DIOEdgeSet			(HBD hbd, unsigned short grp, unsigned short val)	{ BD.DIOGroupEdgeSet(hbd, grp, val); }
void		S626_DIOCapEnableSet		(HBD hbd, unsigned short grp, unsigned short val, unsigned short en)	{ BD.DIOGroupCapEnableSet(hbd, grp, val, en); }
unsigned short  S626_DIOCapEnableGet		(HBD hbd, unsigned short grp)				{ return BD.DIOGroupCapEnableGet(hbd, grp); }
unsigned short  S626_DIOCapStatus		(HBD hbd, unsigned short grp)				{ return BD.DIOGroupCapStatus(hbd, grp); }
void		S626_DIOCapReset		(HBD hbd, unsigned short grp, unsigned short val)	{ BD.DIOGroupCapReset(hbd, grp, val); }
unsigned short  S626_DIOIntEnableGet		(HBD hbd, unsigned short grp)				{ return BD.DIOGroupIntEnableGet(hbd, grp); }
void		S626_DIOIntEnableSet		(HBD hbd, unsigned short grp, unsigned short val)	{ BD.DIOGroupIntEnableSet(hbd, grp, val); }
unsigned short  S626_DIOModeGet			(HBD hbd, unsigned short grp)				{ return BD.DIOChanModeGet(hbd, grp); }
void		S626_DIOModeSet			(HBD hbd, unsigned short grp, unsigned short val)	{ BD.DIOChanModeSet(hbd, grp, val); }

// Counter functions:
DWORD       S626_CounterReadLatch		(HBD hbd, unsigned short ch)				{ return BD.CounterReadLatch(hbd, ch); }
void		S626_CounterCapFlagsReset	(HBD hbd, unsigned short ch)				{ BD.CounterResetCapFlags(hbd, ch); }
unsigned short  S626_CounterCapStatus		(HBD hbd)						{ return BD.CounterCapStatus(hbd); }
unsigned short  S626_CounterModeGet		(HBD hbd, unsigned short ch)				{ return BD.CounterModeGet(hbd, ch); }
void		S626_CounterModeSet		(HBD hbd, unsigned short ch, unsigned short opt)	{ BD.CounterModeSet(hbd, ch, opt); }
void		S626_CounterLatchSourceSet	(HBD hbd, unsigned short ch, unsigned short val)	{ BD.CounterLatchSrcSet(hbd, ch, val); }
void		S626_CounterEnableSet		(HBD hbd, unsigned short ch, unsigned short en)		{ BD.CounterEnableSet(hbd, ch, en); }
void		S626_CounterLoadTrigSet		(HBD hbd, unsigned short ch, unsigned short trg)	{ BD.CounterLoadTrigSet(hbd, ch, trg); }
void		S626_CounterSoftIndex		(HBD hbd, unsigned short ch)				{ BD.CounterSoftIndex(hbd, ch); }
void		S626_CounterPreload		    (HBD hbd, unsigned short ch, U32 val)		{ BD.CounterPreload(hbd, ch, val); }
void		S626_CounterIntSourceSet	(HBD hbd, unsigned short ch, unsigned short src)	{ BD.CounterIntSourceSet(hbd, ch, src); }

// Status and control functions:
U32  	S626_GetAddress			(HBD hbd)						{ return BD.GetAddress(hbd); }
U32  	S626_GetErrors			(HBD hbd)						{ return BD.GetErrors(hbd); }
void		S626_SetErrCallback		(HBD hbd, FPTR_ERR Callback)				{ BD.SetErrCallback(hbd, Callback); }
void		S626_InterruptStatus		(HBD hbd, unsigned short *IntFlags)			{ BD.InterruptStatus(hbd, IntFlags); }
void		S626_InterruptEnable		(HBD hbd, unsigned short en)				{ BD.InterruptEnable(hbd, en); }

void shit(void){printf("shit");}

// Board Open/Close functions:

void S626_CloseBoard( HBD hbd )
{
	// Abort if a valid board handle is not specified.
	if ( hbd >= MAX_BOARDS )
		return;

	// Close the board if it is open.
	Destructor( hbd );		// including: Close the board at CORE level, using S626CORE_BoardClose( hbd );

	// Close the board at CORE level too.
	S626CORE_CoreClose( hbd );

	// Decrease counts of opening boards.
	open_boards --;

}

static int S626_VerifyTypes(void );

void S626_OpenBoard( HBD hbd, DWORD PhysLoc, FPTR_ISR Callback, DWORD Priority )
{
    if( S626_VerifyTypes() != 0) {
        printf("warning: possible types problem");
    }
	// Abort if a valid board handle is not specified.
	if ( hbd >= MAX_BOARDS )
		return;

	// Increase counts of opening boards.
	open_boards ++;

	// Destroy any object that is currently occupying the board handle.
	Destructor( hbd );		// including: Close the board if it is open at CORE level, using S626CORE_BoardClose( hbd );

	// Open CORE object for board open preparation.
	if ( !S626CORE_CoreOpen( hbd, open_boards ) ) 
	{
		S626CORE_SetErrors( hbd, ERR_OPEN );
		return;
	}

	// Attempt to open the board.
	Constructor( hbd, PhysLoc, Callback, Priority, 0 );	// including: Open the board at CORE level, using S626CORE_BoardOpen( hbd, IsBoardRevA );

	//if ( S626CORE_GetErrors( HBD bd ) & ( ERR_OPEN | ERR_CARDREG | ERR_ALLOC_MEMORY | ERR_LOCK_BUFFER | ERR_THREAD | ERR_INTERRUPT ) )
	//printf("error message: ? \n");
	
}

static char dbgfmtmsg[] = "size of %s not correct.  Please fix s626core.h for your architecture\n";

static int S626_VerifyTypes(void)
{
  if( sizeof( U8) != 1) {
    printf(dbgfmtmsg, "U8");
    return -1;
  }
  if( sizeof( U16) != 2) {
    printf(dbgfmtmsg, "U16");
    return -2;
  }
  if( sizeof( U32) != 4) {
    printf(dbgfmtmsg, "U32");
    return -4;
  }
  return 0;
}

unsigned int S626MOD_cpu_to_le32( PVOID addr) 
{
    //TODO  bigendian conversion in user space on BE machines
  return (unsigned int) addr;
}
