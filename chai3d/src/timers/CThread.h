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
#ifndef CThreadH
#define CThreadH
//---------------------------------------------------------------------------
#include "extras/CGlobals.h"
//---------------------------------------------------------------------------

enum CThreadPriority
{
    CHAI_THREAD_PRIORITY_GRAPHICS,
    CHAI_THREAD_PRIORITY_HAPTICS
};

//===========================================================================
/*!
    \file   CThread.h
	\class	cThread
	\brief	cThread provides a class to manage threads.
*/
//===========================================================================
class cThread
{
  public:
    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cThread.
    cThread();

    //! Destructor of cThread.
    ~cThread();

    // METHODS:
    //! set the thread parameters.
    void set(void (*a_function)(void), CThreadPriority a_level);

    //! set the thread priority level.
    void setPriority(CThreadPriority a_level);

    //! get the current thread priority level
    CThreadPriority getPriority() { return (m_priorityLevel); }

  protected:

#if defined(_WIN32)
    //! thread handle
    DWORD m_threadId;
#endif

#if defined(_LINUX) || defined(_APPLE)
    // thread handle
    pthread_t m_handle;
#endif

    // pointer to thread function
    void* m_function;

    // thread priority level
    CThreadPriority m_priorityLevel;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
