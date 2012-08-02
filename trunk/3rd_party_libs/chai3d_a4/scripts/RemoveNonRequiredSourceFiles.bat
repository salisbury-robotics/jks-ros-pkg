rem ===========================================================================
rem
rem    CHAI 3D    
rem    
rem    Author:	  Francois Conti
rem    		  conti@chai3d.org
rem    
rem    Purpose:	  Removes sources files which are not necessary in the 
rem               various releases
rem
rem ===========================================================================

rem -----------------------------------------
rem INITIALISATION
rem -----------------------------------------

echo off

path
cd ../..


rem -----------------------------------------
rem WIN32
rem -----------------------------------------
cd win32/src

erase /F /Q devices\dhdc.h

cd ../..


rem -----------------------------------------
rem WIN64
rem -----------------------------------------
cd win64/src

erase /F /Q devices\App626.h
erase /F /Q devices\CDriverSensoray626.h
erase /F /Q devices\CDriverSensoray626.cpp
erase /F /Q devices\CDriverServotogo.h
erase /F /Q devices\CDriverServotogo.cpp
erase /F /Q devices\DLPORTIO.H
erase /F /Q devices\dhdc.h

cd ../..


rem -----------------------------------------
rem LINUX
rem -----------------------------------------
cd linux/src

rmdir /S /Q display
erase /F /Q devices\App626.h
erase /F /Q devices\CDriverSensoray626.h
erase /F /Q devices\CDriverSensoray626.cpp
erase /F /Q devices\CDriverServotogo.h
erase /F /Q devices\CDriverServotogo.cpp
erase /F /Q devices\CFalconDevice.h
erase /F /Q devices\CFalconDevice.cpp
erase /F /Q devices\CFreedom6SDevice.h
erase /F /Q devices\CFreedom6SDevice.cpp
erase /F /Q devices\CVirtualDevice.h
erase /F /Q devices\CVirtualDevice.cpp
erase /F /Q devices\DLPORTIO.H
erase /F /Q devices\WIN626.C
erase /F /Q devices\WIN626.h

cd ../..


rem -----------------------------------------
rem MAC
rem -----------------------------------------
cd mac/src

rmdir /S /Q display
erase /F /Q devices\App626.h
erase /F /Q devices\CDriverSensoray626.h
erase /F /Q devices\CDriverSensoray626.cpp
erase /F /Q devices\CDriverServotogo.h
erase /F /Q devices\CDriverServotogo.cpp
erase /F /Q devices\CFalconDevice.h
erase /F /Q devices\CFalconDevice.cpp
erase /F /Q devices\CFreedom6SDevice.h
erase /F /Q devices\CFreedom6SDevice.cpp
erase /F /Q devices\CPhantomDevices.h
erase /F /Q devices\CPhantomDevices.cpp
erase /F /Q devices\CVirtualDevice.h
erase /F /Q devices\CVirtualDevice.cpp
erase /F /Q devices\DLPORTIO.H
erase /F /Q devices\WIN626.C
erase /F /Q devices\WIN626.h

cd ../..
