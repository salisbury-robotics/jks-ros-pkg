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
#ifndef CPrecisionClockH
#define CPrecisionClockH
//---------------------------------------------------------------------------
#ifdef _WIN32
#include "windows.h"
#endif
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file   CPrecisionClock.h
	\class	cPrecisionClock
	\brief	cPrecisionClock provides a class to manage high-precision
			time measurments.  All measurements are in seconds unless
            otherwise-specified.
*/
//===========================================================================
class cPrecisionClock
{
  public:
    // CONSTRUCTOR & DESTRUCTOR:
    //! Constructor of cPrecisionClock.
    cPrecisionClock();

    //! Destructor of cPrecisionClock.
    ~cPrecisionClock();

    // METHODS:
    //! reset clock to zero.
    void reset();

    //! start counting time; optionally reset the clock to zero
    double start(bool resetClock=false);

    //! stop counting time; return the elapsed time
    double stop();

    //! return \b true if timer is currently \b on, else return \b false.
    bool on() { return (m_on); };

    //! read the current clock time (seconds) (the time that has elapsed since the last call to "start")
    double getCurrentTimeSeconds();

    //! set the period before a "timeout" occurs (you need to poll for this)
    void setTimeoutPeriodSeconds(double a_timeoutPeriod);

    //! read the programmed timeout period
    double getTimeoutPeriodSeconds() { return (m_timeoutPeriod); }

    //! returns \b true if a timeout has occurred
    bool timeoutOccurred();

    //! returns \b true if high resolution timers are available on this computer.
    bool highResolution() { return (m_highres); };

    //! If all you want is something that tells you the time, this is your function...
    double getCPUTimeSeconds();
    
    //! For backwards-compatibility...
    double getCPUTime() { return getCPUTimeSeconds(); }
    double getCPUtime() { return getCPUTimeSeconds(); }

  private:

#ifdef _WIN32
    //! Stores information about CPU high precision clock.
    LARGE_INTEGER m_freq;
#endif

    //! Time accumulated between previous calls to "start" and "stop"
    double m_timeAccumulated;

    //! CPU time when clock was started.
    double m_timeStart;

    //! Timeout period
    double m_timeoutPeriod;

    //! clock time when timer was started.
    double m_timeoutStart;

    //! If \b true, a  high precision CPU clock is available.
    bool m_highres;
    
    //! If \b true, the clock is \b on.
    bool m_on;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
