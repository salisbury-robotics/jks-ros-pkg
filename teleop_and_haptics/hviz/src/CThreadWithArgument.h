#ifndef CTHREADWITHARGUMENT_H
#define CTHREADWITHARGUMENT_H

#include <chai3d.h>

class cThreadWithArgument :
    public cThread
{
public:
    cThreadWithArgument() : cThread()   { }
    virtual ~cThreadWithArgument()      { }

    void set(void (*a_function)(void *), void *a_argument, CThreadPriority a_level)
    {
        // create thread
#if defined(_WIN32)
        CreateThread(
              0,
              0,
              (LPTHREAD_START_ROUTINE)(a_function),
              a_argument,
              0,
              &m_threadId
          );
#endif

#if defined (_LINUX) || defined (_MACOSX)
        pthread_create(
              &m_handle,
              0,
              (void * (*)(void*)) a_function,
              a_argument
        );
#endif

        // set thread priority level
        setPriority(a_level);
    }

};

#endif
