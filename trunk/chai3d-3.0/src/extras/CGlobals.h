//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2012, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 699 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGlobalsH
#define CGlobalsH
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGlobals.h
    \ingroup    extras

    \brief  
    <b> Extras </b> \n 
	General CHAI3D Settings.
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------


//===========================================================================
// GENERAL
//===========================================================================

//! maximum length of a path
#define CHAI_SIZE_PATH		255

//! maximum length of a object name
#define CHAI_SIZE_NAME		64

// standard libraries
#include <cstdlib>
#include <cstring>


//===========================================================================
// WIN32 / WIN64 OS
//===========================================================================
#if defined(WIN32) | defined(WIN64)

    //--------------------------------------------------------------------
    // GENERAL
    //--------------------------------------------------------------------

    // OS specific
    #include "windows.h"

    // printf
    #include <conio.h>
    #define CHAI_DEBUG_PRINT _cprintf

    //--------------------------------------------------------------------
    // HAPTIC DEVICES
    //--------------------------------------------------------------------
    #define _ENABLE_CUSTOM_DEVICE_SUPPORT
    #define _ENABLE_DELTA_DEVICE_SUPPORT
    #define _ENABLE_FALCON_DEVICE_SUPPORT
    #define _ENABLE_MPB_DEVICE_SUPPORT
    #define _ENABLE_PHANTOM_DEVICE_SUPPORT
    #define _ENABLE_SENSORAY626_DEVICE_SUPPORT
    #define _ENABLE_SERVOTOGO_DEVICE_SUPPORT
    #define _ENABLE_VIRTUAL_DEVICE_SUPPORT

#endif


//===========================================================================
// LINUX OS
//===========================================================================
#if defined(LINUX)

    //--------------------------------------------------------------------
    // GENERAL
    //--------------------------------------------------------------------

    // OS specific
    #include <sys/time.h>

    // threads
    #include "pthread.h"

    // printf
    #define CHAI_DEBUG_PRINT printf

    //--------------------------------------------------------------------
    // HAPTIC DEVICES
    //--------------------------------------------------------------------
    #define _ENABLE_CUSTOM_DEVICE_SUPPORT
    // (support for Force Dimension devices is handled in the Makefile structure)
    // #define _ENABLE_DELTA_DEVICE_SUPPORT

#endif


//===========================================================================
// MAC OS
//===========================================================================
#if defined(MACOSX)

    //--------------------------------------------------------------------
    // GENERAL
    //--------------------------------------------------------------------

    // OS specific
    #include <mach/mach_time.h>
    #include <mach/kern_return.h>

    // threads
    #include "pthread.h"

    // printf
    #define CHAI_DEBUG_PRINT printf

    //--------------------------------------------------------------------
    // HAPTIC DEVICES
    //--------------------------------------------------------------------
    #define _ENABLE_CUSTOM_DEVICE_SUPPORT
    // (support for Force Dimension devices is handled in the Makefile structure)
    // #define _ENABLE_DELTA_DEVICE_SUPPORT

#endif


//---------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
