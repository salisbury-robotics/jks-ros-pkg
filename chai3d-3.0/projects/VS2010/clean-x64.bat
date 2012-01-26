@echo off

REM  This file is part of the CHAI 3D visualization and haptics libraries.
REM  Copyright (C) 2003-2010 by CHAI 3D. All rights reserved.
REM  
REM  This library is free software; you can redistribute it and/or modify
REM  it under the terms of the GNU General Public License("GPL") version 2
REM  as published by the Free Software Foundation.
REM  
REM  For using the CHAI 3D libraries with software that can not be combined
REM  with the GNU GPL, and for taking advantage of the additional benefits
REM  of our support services, please contact CHAI 3D about acquiring a
REM  Professional Edition License.
REM  
REM  \author    <http://www...\..\CHAI3D-VS2008.org>
REM  \author    Sebastien Grange
REM  \version   $Rev: 616 $


set cmp=none
if exist "%ProgramFiles(x86)%\microsoft visual studio 10.0\common7\ide\devenv.exe"    set cmp=devenv
if exist "%ProgramFiles(x86)%\microsoft visual studio 10.0\common7\ide\VCExpress.exe" set cmp=VCExpress
if exist "%ProgramFiles%\microsoft visual studio 10.0\common7\ide\devenv.exe"         set cmp=devenv
if exist "%ProgramFiles%\microsoft visual studio 10.0\common7\ide\VCExpress.exe"      set cmp=VCExpress

if %cmp% == none goto SKIP

set OLDPATH=%PATH%
if exist "%ProgramFiles(x86)%\microsoft visual studio 10.0\common7\ide" set PATH="%ProgramFiles(x86)%\microsoft visual studio 10.0\common7\ide";%PATH%
if exist "%ProgramFiles%\microsoft visual studio 10.0\common7\ide" set PATH="%ProgramFiles%\microsoft visual studio 10.0\common7\ide";%PATH%

%cmp% /clean "Debug|x64"   "..\..\CHAI3D-VS2010.sln" > clean.txt
%cmp% /clean "Release|x64" "..\..\CHAI3D-VS2010.sln" > clean.txt
goto done

:SKIP
echo     * warning - compiler missing, skipping
goto :EOF

:done
set PATH=%OLDPATH%

:EOF
