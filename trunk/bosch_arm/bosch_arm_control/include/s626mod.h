///////////////////////////////////////////////////////////////////////////////////////////////////
// Module : s626mod.h Function : Declarations of type define used in
// driver and functions in 626 module-level code  that are called by
// the core.  Copyright (C) Sensoray Company 2004
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DEF_626MOD
#define DEF_626MOD

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DEF_626DRV
#include "s626types.h"
// Type definitions:


//data structures for DMA handling.
typedef struct
{
	void *PhysicalBase;
	void *LogicalBase;
	DWORD DMAHandle;
} DMABUF;

#endif	//#ifndef DEF_626DRV


#include "App626.h"					// Model 626 application definitions.


////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////  OS-dependent FUNCTIONS  ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

// OS-dependent functions, which are basically wrappers for kernel-supplied functions.
// They must be implemented in the module-level code:

//		in user space:
void	S626MOD_CriticalBegin		( HBD hbd );						// Overall critical section begin.
void	S626MOD_CriticalEnd			( HBD hbd );						// Overall critical section end.
void	S626MOD_CriticalBeginAdc	( HBD hbd );						// ADC critical section begin.
void	S626MOD_CriticalEndAdc		( HBD hbd );						// ADC critical section end.
DWORD	S626MOD_ReadMem				( PVOID address );
void	S626MOD_WriteMem			( PVOID address, DWORD data );
void	S626MOD_AllocDMAB			( HBD hbd, DWORD nBytes, DMABUF *pDmaBuf );	// Allocate DMA buffer.
void	S626MOD_CloseDMAB			( HBD hbd, DMABUF *pDmaBuf );				// Release DMA buffer.

//		in kernel space:
void	S626MOD_sleep				( int milliseconds );
void	S626MOD_delay_usec			( DWORD microseconds );
WORD	S626MOD_read_io16			( int addr );
void	S626MOD_write_io16			( int addr, WORD data );
DWORD 	S626MOD_read_mem32			( int addr );
void	S626MOD_write_mem32			( int addr, DWORD data );
void	S626MOD_block_read_io16		( int addr, WORD *buf, int nwords );
void	S626MOD_block_write_io16	( int addr, WORD *buf, int nwords );
DWORD 	S626MOD_time_elapsed		( DWORD start_time );
DWORD 	S626MOD_time_now			( void );
int		S626MOD_copy_to_user		( char *dst_buffer, const char *src_buffer, int nbytes );
int		S626MOD_copy_from_user		( char *dst_buffer, const char *src_buffer, int nbytes );
DWORD 	S626MOD_dmabuf_alloc		( void );
void	S626MOD_dmabuf_free			( DWORD virt_adrs );
unsigned int S626MOD_cpu_to_le32             (void *paddr );

#ifdef __cplusplus
}
#endif


#endif	// #ifndef DEF_626MOD
