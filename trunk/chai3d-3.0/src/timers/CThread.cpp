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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 710 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "CThread.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cThread.

    \fn		cThread::cThread()
*/
//===========================================================================
cThread::cThread()
{
    // no thread function has been defined yet
    m_function = 0;

    // default value for priority level
    m_priorityLevel = CTHREAD_PRIORITY_GRAPHICS;
}


//===========================================================================
/*!
    Destructor of cThread.

    \fn		cThread::~cThread()
*/
//===========================================================================
cThread::~cThread()
{
}


//===========================================================================
/*!
    Creates a thread to execute within the address space of the calling process.
    Parameters include a pointer to the function and its priority level.

    \fn		void cThread::set(void (*a_function)(void), CThreadPriority a_level)
    \param  a_function Pointer to thread function
    \param  a_level Priority level of thread.
*/
//===========================================================================
void cThread::set(void(*a_function)(void), CThreadPriority a_level)
{
    // create thread
#if defined(WIN32) | defined(WIN64)
    CreateThread(
          0,
          0,
          (LPTHREAD_START_ROUTINE)(a_function),
          0,
          0,
          &m_threadId
      );
#endif

#if defined (LINUX) || defined (MACOSX)
    pthread_create(
          &m_handle,
          0,
          (void * (*)(void*)) a_function,
          0
    );
#endif

    // set thread priority level
    setPriority(a_level);
}


//===========================================================================
/*!
    Creates a thread to execute within the address space of the calling process.
    Parameters include a pointer to the function and its priority level.

    \fn     void cThread::set(void (*a_function)(void), CThreadPriority a_level)
    \param  a_function Pointer to thread function
    \param  a_level Priority level of thread.
*/
//===========================================================================
void cThread::set(void(*a_function)(void*), CThreadPriority a_level, void *arg)
{
    // create thread
#if defined(WIN32) | defined(WIN64)
    CreateThread(
          0,
          0,
          (LPTHREAD_START_ROUTINE)(a_function),
          arg,
          0,
          &m_threadId
      );
#endif

#if defined (LINUX) || defined (MACOSX)
    pthread_create(
          &m_handle,
          0,
          (void * (*)(void*)) a_function,
          arg
    );
#endif

    // set thread priority level
    setPriority(a_level);
}


//===========================================================================
/*!
    Adjust the priority level of the thread.

    \fn		void cThread::setPriority(CThreadPriority a_level)
    \param  a_level  Priority level of the thread
*/
//===========================================================================
void cThread::setPriority(CThreadPriority a_level)
{
    m_priorityLevel = a_level;

#if defined(WIN32) | defined(WIN64)
    switch (m_priorityLevel)
    {
        case CTHREAD_PRIORITY_GRAPHICS:
        SetThreadPriority(&m_threadId, THREAD_PRIORITY_NORMAL);
        break;

        case CTHREAD_PRIORITY_HAPTICS:
        SetThreadPriority(&m_threadId, THREAD_PRIORITY_ABOVE_NORMAL);
        break;
    }
#endif

#if defined(LINUX) || defined(MACOSX)
    struct sched_param sp;
    memset(&sp, 0, sizeof(struct sched_param));

    switch (m_priorityLevel)
    {
        case CTHREAD_PRIORITY_GRAPHICS:
        sp.sched_priority = 5;
        break;

        case CTHREAD_PRIORITY_HAPTICS:
        sp.sched_priority = 10;
        break;
    }

    pthread_setschedparam(m_handle, SCHED_RR, &sp);
#endif
}
