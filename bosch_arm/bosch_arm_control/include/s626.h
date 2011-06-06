//////////////////////////////////////////////////////////////////////////////
// Module   : s626.h                                                        //
// Product  : Sensoray 626 Linux driver.                                    //
// Function : The s626_ioctl() commands.                                    //
//                                                                          //
// Examples:                                                                //
//    s626_ioctl( handle, S626_IOC_WriteRegister, data );                   //
//    br = s626_ioctl( handle, S626_IOC_GetCardInfo, pcardInfo );           //
//////////////////////////////////////////////////////////////////////////////

#ifndef __s626__h
#define __s626__h

//===============================================================================================================
// Register Write/Read.

#define S626_IOC_WriteRegister			0x0101		// Write data to register.		unsigned int
#define S626_IOC_ReadRegister			0x0102		// Read data from register.		unsigned int

//===============================================================================================================
// Registration.

#define S626_IOC_Register				0x0201		// unsigned int
#define S626_IOC_Unregister				0x0202		// SX_CARD_REGISTER*
#define S626_IOC_GetCardInfo			0x0203		// SX_PCI_CARD_INFO*

//===============================================================================================================
// Open/Close.

#define S626_IOC_Init					0x0301		// unsigned int
#define S626_IOC_Close					0x0302		// unsigned int

//===============================================================================================================
// Interrupts.

#define S626_IOC_RequestIrq				0x0401		// SX_INTERRUPT*
#define S626_IOC_InterruptOn			0x0402		// unsigned int
#define S626_IOC_InterruptOff			0x0403		// unsigned int

//===============================================================================================================
// DMA allocation.

#define S626_IOC_AllocDMAB				0x0501		// ioc_param*
#define S626_IOC_CloseDMAB				0x0502		// ioc_param*


#endif


