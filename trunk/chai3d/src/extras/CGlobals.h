//===========================================================================
/*
    This file is part of the CHAI 3D visualization and haptics libraries.
    Copyright (C) 2003-#YEAR# by CHAI 3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI 3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI 3D about acquiring a
    Professional Edition License.

    \author:    <http://www.chai3d.org>
    \author:    Francois Conti
    \version    #CHAI_VERSION#
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGlobalsH
#define CGlobalsH
//---------------------------------------------------------------------------

//===========================================================================
// WINDOWS OS
//===========================================================================
#ifdef _WIN32
    //--------------------------------------------------------------------
    // GENERAL
    //--------------------------------------------------------------------
    // general windows
    #include "windows.h"

    // This needs to happen before GLUT gets included
    #include <stdlib.h>

    //--------------------------------------------------------------------
    // HAPTIC DEVICES
    //--------------------------------------------------------------------


    //--------------------------------------------------------------------
    // BBCP - BORLAND BUILDER
    //--------------------------------------------------------------------
    #ifdef _BBCP

        // printf
        #define CHAI_DEBUG_PRINT printf

        // open gl
        #include "GL/glut.h"
        #include "GL/gl.h"
    #endif

    //--------------------------------------------------------------------
    // MSVC - MICROSOFT VISUAL STUDIO
    //--------------------------------------------------------------------
    #ifdef _MSVC

        // turn off annoying compiler warnings
        #pragma warning(disable: 4305)
        #pragma warning(disable: 4800)
        #pragma warning(disable: 4786)

        // printf
        #include <conio.h>
        #define CHAI_DEBUG_PRINT _cprintf

        // open gl
        #include "gl/glut.h"

    #endif

#endif

//===========================================================================
// LINUX OS
//===========================================================================
#ifdef _LINUX

	// printf
	#define CHAI_DEBUG_PRINT printf

	// standard libraries
	#include <stdlib.h>
	#include <string.h>

	// open gl
	#include "gl/gl.h"
	#include "gl/glut.h"

	// threads
	#include "pthread.h"	

	// devices
    #define _DISABLE_FALCON_DEVICE_SUPPORT
	#define _DISABLE_PHANTOM_DEVICE_SUPPORT
	#define _DISABLE_MPB_DEVICE_SUPPORT
	#define _DISABLE_VIRTUAL_DEVICE_SUPPORT

#endif

//===========================================================================
// MAC OS
//===========================================================================
#ifdef _APPLE

	// printf
	#define CHAI_DEBUG_PRINT printf

	// standard libraries
	#include <stdlib.h>
	#include <string.h>

	// open gl
	#include "GLUT/glut.h"

	// threads
	#include "pthread.h"	

	// devices
	#define _DISABLE_FALCON_DEVICE_SUPPORT
	#define _DISABLE_PHANTOM_DEVICE_SUPPORT
	#define _DISABLE_MPB_DEVICE_SUPPORT
	#define _DISABLE_VIRTUAL_DEVICE_SUPPORT
	
    // TODO: remove this when we get Delta libs going
    #define _DISABLE_DELTA_DEVICE_SUPPORT

#endif


//===========================================================================
// GENERAL
//===========================================================================
// maximum length of a path
#define CHAI_SIZE_PATH 255

// maximum length of a object name
#define CHAI_SIZE_NAME 64

// a large double
#define CHAI_DBL_MAX 9999999


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

