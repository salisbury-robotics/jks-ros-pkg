///////////////////////////////////////////////////////////////////////////////
//  Module        : s626api.h
//  Function      : Header file for Sensoray 626 applications.
//  Usage         : all apps using 626 boards must include this file
//  Author        : David Stroupe and Charlie X. Liu
//  Copyright     : (C) 2002 - 2004 Sensoray Co., Inc.
///////////////////////////////////////////////////////////////////////////////
//* This file contains all the function prototypes of 626 linux driver API.
//* Since the API function prototypes is the same as the prototypes for 
//* Windows, refer to the manual <Model 626 Driver for Windows>, for the 
//* reference of programming.
///////////////////////////////////////////////////////////////////////////////

#include "s626types.h"
VOID                      S626_OpenBoard ( HBD, DWORD, FPTR_ISR, DWORD );
VOID                      S626_CloseBoard ( HBD );
DWORD                     S626_GetAddress ( HBD );
DWORD                     S626_GetErrors ( HBD );
WORD                      S626_RegRead ( HBD, WORD );
VOID                      S626_RegWrite ( HBD, WORD, WORD );
VOID                      S626_ResetADC ( HBD, BYTE * );
VOID                      S626_StartADC ( HBD );
VOID                      S626_WaitDoneADC ( HBD , WORD * );
VOID                      S626_ReadADC ( HBD, WORD * );
VOID                      S626_WriteDAC ( HBD, WORD, LONG );
VOID                      S626_DIOModeSet ( HBD, WORD, WORD );
WORD                      S626_DIOModeGet ( HBD, WORD );
VOID                      S626_DIOWriteBankSet ( HBD, WORD, WORD );
WORD                      S626_DIOWriteBankGet ( HBD, WORD );
WORD                      S626_DIOReadBank ( HBD, WORD );
VOID                      S626_DIOEdgeSet ( HBD, WORD, WORD );
WORD                      S626_DIOEdgeGet ( HBD, WORD );
VOID                      S626_DIOCapEnableSet ( HBD, WORD, WORD, WORD );
WORD                      S626_DIOCapEnableGet ( HBD, WORD );
WORD                      S626_DIOCapStatus ( HBD, WORD );
VOID                      S626_DIOCapReset ( HBD, WORD, WORD );
VOID                      S626_DIOIntEnableSet ( HBD, WORD, WORD );
WORD                      S626_DIOIntEnableGet ( HBD, WORD );
VOID                      S626_CounterModeSet ( HBD, WORD, WORD );
WORD                      S626_CounterModeGet ( HBD, WORD );
VOID                      S626_CounterEnableSet ( HBD, WORD, WORD );
VOID                      S626_CounterPreload ( HBD, WORD, DWORD );
VOID                      S626_CounterLoadTrigSet ( HBD, WORD, WORD );
VOID                      S626_CounterLatchSourceSet ( HBD, WORD, WORD );
DWORD                     S626_CounterReadLatch ( HBD, WORD );
WORD                      S626_CounterCapStatus ( HBD );
VOID                      S626_CounterCapFlagsReset ( HBD, WORD );
VOID                      S626_CounterSoftIndex ( HBD, WORD );
VOID                      S626_CounterIntSourceSet ( HBD, WORD, WORD );
VOID                      S626_WatchdogPeriodSet ( HBD, WORD );
WORD                      S626_WatchdogPeriodGet ( HBD );
VOID                      S626_WatchdogEnableSet ( HBD, WORD );
WORD                      S626_WatchdogEnableGet ( HBD );
VOID                      S626_WatchdogReset ( HBD );
WORD                      S626_WatchdogTimeout ( HBD );
VOID                      S626_BackupEnableSet ( HBD, WORD );
WORD                      S626_BackupEnableGet ( HBD );
VOID                      S626_ChargeEnableSet ( HBD, WORD );
WORD                      S626_ChargeEnableGet ( HBD );
VOID                      S626_InterruptEnable ( HBD, WORD );
VOID                      S626_InterruptStatus ( HBD, WORD * );
VOID                      S626_SetErrCallback (HBD, FPTR_ERR);
