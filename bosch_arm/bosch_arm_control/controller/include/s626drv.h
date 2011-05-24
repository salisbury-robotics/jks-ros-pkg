//*********************** 626driver.h **********************************
//* This file contains special types, constants and function prototypes
//* interfacing Sensoray 626 low-level loadable linux device driver.
//* Copyright (C) Sensoray Company 2002-2004
//**********************************************************************

#ifndef DEF_626DRV
#define DEF_626DRV

#include "s626types.h"
#define S626MAGIC 0xDB
#define GOTINTERRUPT 0xDA
#define KILLINTERRUPT 0xAD

#define MAX_626_BOARDS	4

#define S626_MAJOR		146
/*/
#define S626WriteRegister _IOW(S626MAGIC, 0x01, unsigned int)
#define S626ReadRegister _IOWR(S626MAGIC, 0x02, unsigned int)
#define S626AllocDMAB  _IOWR(S626MAGIC, 0x03, driverdata*)
#define S626CloseDMAB  _IOWR(S626MAGIC, 0x04, driverdata*)
#define S626Register _IOWR(S626MAGIC, 0x05, unsigned int)
#define S626RequestIrq _IOWR(S626MAGIC, 0x06, SX_INTERRUPT*)
#define S626Init _IOWR(S626MAGIC, 0x07, unsigned int)
#define S626Close _IOWR(S626MAGIC, 0x08, unsigned int)
#define S626InterruptOn _IOWR(S626MAGIC, 0x09, unsigned int)
#define S626InterruptOff _IOWR(S626MAGIC, 0x0A, unsigned int)
#define S626GetCardInfo _IOWR(S626MAGIC, 0x0B, SX_PCI_CARD_INFO*)
#define S626Unregister _IOWR(S626MAGIC, 0x0C, SX_CARD_REGISTER*)
/*/

// Type definitions:



#define		TRUE			1
#define		FALSE			0

#define	INTERRUPT_LEVEL_SENSITIVE	 1
#define	INTERRUPT_CMD_COPY			 2

#define	INTERRUPT_STATUS_CMDS		32
#define	INTERRUPT_PROCESS_CMDS		 8
#define	SX_CARD_ITEMS				16
#define	SX_PCI_CARDS				32

typedef struct
{
	DWORD            cmdTrans;  // Transfer command SX_TRANSFER_CMD
	DWORD            dwPort;    // io port for transfer or user memory address
	// parameters used for string trasfers:
	DWORD            dwBytes;   // for string transfer
	DWORD            fAutoinc;  // transfer from one port/address
					 // or use incremental range of addresses
	DWORD            dwOptions; // must be 0
	union
	{
		UCHAR        Byte;      // use for byte transfer
		USHORT       Word;      // use for word transfer
		DWORD        Dword;     // use for dword transfer
		PVOID        pBuffer;   // use for string transfer
	} Data;
} SX_TRANSFER;

typedef struct
{
	PVOID        hInterrupt;    // handle of interrupt
	DWORD        dwInterruptNum;// number of interrupt to install
	DWORD        dwInterruptInd;// index of interrupt in internal table
	DWORD        fNotSharable;  // is interrupt unshareable
	DWORD        dwOptions;     // interrupt options: INTERRUPT_LEVEL_SENSITIVE, INTERRUPT_CMD_COPY
	SX_TRANSFER  StatusCmd[INTERRUPT_STATUS_CMDS];   // command to get interrupt status
	DWORD        dwStatusCmds;  // number of commands to get status
	DWORD        dwAndMask;     // to determine our interrupt driver checks:
	DWORD        dwXorMask;     //  Status.Data.Dword & dwAndMask ^ dwXorMask
	SX_TRANSFER  ProcessCmd[INTERRUPT_PROCESS_CMDS]; // commands to do on interrupt
	DWORD        dwProcessCmds; // number of commands to process interrupt
	volatile DWORD  dwCounter;  // number of interrupts received
	volatile DWORD  dwLost;     // number of interrupts not yet dealt with
	volatile DWORD  fStopped;   // was interrupt disabled during wait
} SX_INTERRUPT;

typedef enum { ITEM_NONE=0, ITEM_INTERRUPT=1, ITEM_MEMORY=2, ITEM_IO=3 } ITEM_TYPE;

typedef struct
{
	DWORD item; 	// ITEM_TYPE

	DWORD fNotSharable;

	union
	{
		struct	// ITEM_MEMORY
		{
			PVOID pPhysicalAddr;	// physical address on card
			DWORD dwBytes;			// address range
			PVOID pTransAddr;		// returns the address to pass on to transfer commands
			PVOID pUserDirectAddr;	// returns the address for direct user read/write
		} Mem;
		struct	// ITEM_IO
		{
			//DWORD dwAddr;			// begining of io address
			//DWORD dwBytes;		// io range
			PVOID pPhysicalAddr;	// physical address on card
			DWORD dwBytes;			// address range
		        PVOID pTransAddr;		// returns the address to pass on to transfer commands
			PVOID pUserDirectAddr;	// returns the address for direct user read/write
		} IO;
		struct	// ITEM_INTERRUPT
		{
			DWORD dwInterrupt;		// number of interrupt to install
			struct
			{
				DWORD dwLevel;
				DWORD dwSens;
			} Options;				// interrupt options: INTERRUPT_LEVEL_SENSITIVE
			PVOID hInterrupt;		// returns the handle of the interrupt installed
		} Int;
		struct
		{
			DWORD dw1;
			DWORD dw2;
			DWORD dw3;
			DWORD dw4;
			DWORD dw5;
		} Val;
	} I;
} SX_ITEMS;

typedef struct
{
	DWORD    dwItems;
	SX_ITEMS Item[SX_CARD_ITEMS];
} SX_CARD;

typedef struct
{
	SX_CARD Card;           // card to register
	DWORD   fCheckLockOnly; // only check if card is lockable, return hCard=1 if OK
	PVOID   hCard;          // handle of card
} SX_CARD_REGISTER;

typedef struct
{
	DWORD dwBus;
	DWORD dwSlot;
	DWORD dwFunction;
} SX_PCI_SLOT;

typedef struct
{
	DWORD dwVendorId;
	DWORD dwDeviceId;
} SX_PCI_ID;

typedef struct
{
	SX_PCI_ID   searchId;	// if dwVendorId==0 - scan all vendor IDs
							// if dwDeviceId==0 - scan all device IDs
	DWORD dwCards;			// number of cards found
	SX_PCI_ID   cardId[SX_PCI_CARDS];     // VendorID & DeviceID of cards found
	SX_PCI_SLOT cardSlot[SX_PCI_CARDS];   // pci slot info of cards found
} SX_PCI_SCAN_CARDS;

typedef struct
{
	SX_PCI_SLOT pciSlot;    // pci slot
	SX_CARD     Card;       // get card parameters for pci slot
	DWORD       dwDevNum;   // number of device in internal table
} SX_PCI_CARD_INFO;

#ifndef PCI_TYPE0_ADDRESSES

#define PCI_TYPE0_ADDRESSES	6
#define PCI_TYPE1_ADDRESSES	2

typedef struct _PCI_COMMON_CONFIG
{
	USHORT  VendorID;
	USHORT  DeviceID;
	USHORT  Command;
	USHORT  Status;
	UCHAR   RevisionID;
	UCHAR   ProgIf;
	UCHAR   SubClass;
	UCHAR   BaseClass;
	UCHAR   CacheLineSize;
	UCHAR   LatencyTimer;
	UCHAR   HeaderType;
	UCHAR   BIST;                       // Built in self test

	union
	{
		struct _PCI_HEADER_TYPE_0
		{
			ULONG   BaseAddresses[PCI_TYPE0_ADDRESSES];
			ULONG   CIS;
			USHORT  SubVendorID;
			USHORT  SubSystemID;
			ULONG   ROMBaseAddress;
			UCHAR   CapabilitiesPtr;
			UCHAR   Reserved1[3];
			ULONG   Reserved2;
			UCHAR   InterruptLine;
			UCHAR   InterruptPin;
			UCHAR   MinimumGrant;
			UCHAR   MaximumLatency;
		} type0;
	} u;
	UCHAR   DeviceSpecific[192];
} PCI_COMMON_CONFIG, *PPCI_COMMON_CONFIG;

#endif

typedef struct
{
	SX_PCI_SLOT pciSlot;    // pci bus, slot and function number
	PCI_COMMON_CONFIG pciConfig;    // buffer for read/write
	DWORD       dwOffset;   // offset in pci configuration space to read/write from
	DWORD       fIsRead;    // if 1 then read pci config, 0 write pci config
	DWORD       dwResult;   // returns the number of bytes of data if read/write ok
				//         0 the specified PCI bus does not exist
				//         2 The specified PCI bus exists,
				//		   but there is no device at the given PCI SlotNumber
} SX_PCI_CONFIG_DUMP;


typedef struct
{
	void *PhysicalBase;
	void *LogicalBase;
	DWORD DMAHandle;
} DMABUF;


//data structures for driver communciation through ioctl().
typedef struct {
	  short	boardhandle;
	  PVOID	base;
	  PVOID	address;
	  DWORD	data;
	  void	*PhysicalBase;
	  void	*LogicalBase;
	  DWORD	DMAHandle;
} ioc_param;



#endif	//#define DEF_626DRV

