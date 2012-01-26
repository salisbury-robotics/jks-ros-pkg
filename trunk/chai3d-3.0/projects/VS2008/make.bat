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
REM  \version   $Rev: 519 $


set cmp=none
if exist "%ProgramFiles(x86)%\microsoft visual studio 9.0\common7\ide\devenv.exe"    set cmp=devenv
if exist "%ProgramFiles(x86)%\microsoft visual studio 9.0\common7\ide\VCExpress.exe" set cmp=VCExpress
if exist "%ProgramFiles%\microsoft visual studio 9.0\common7\ide\devenv.exe"         set cmp=devenv
if exist "%ProgramFiles%\microsoft visual studio 9.0\common7\ide\VCExpress.exe"      set cmp=VCExpress

if %cmp% == none goto SKIP

set OLDPATH=%PATH%
if exist "%ProgramFiles(x86)%\microsoft visual studio 9.0\common7\ide" set PATH="%ProgramFiles(x86)%\microsoft visual studio 9.0\common7\ide";%PATH%
if exist "%ProgramFiles%\microsoft visual studio 9.0\common7\ide" set PATH="%ProgramFiles%\microsoft visual studio 9.0\common7\ide";%PATH%

if %1a == a        goto Release
if %1  == /c       goto clean
if %1  == clean    goto clean
if %1  == Release  goto Release
if %1  == release  goto Release
if %1  == /f       goto ReleaseForced
if %1  == force    goto ReleaseForced
if %1  == Debug    goto Debug
if %1  == debug    goto Debug

:Release
if %2a == a goto ReleaseBuild
if %2 == /f goto ReleaseForced
goto done

:ReleaseBuild
%cmp% /build "Release|Win32" "..\..\CHAI3D-VS2008.sln" > output.txt
goto done

:ReleaseForced
%cmp% /rebuild "Release|Win32" "..\..\CHAI3D-VS2008.sln"  > output.txt
goto done

:Debug
if %2a == a goto DebugBuild
if %2 == /f goto DebugForced
goto done

:DebugBuild
%cmp% /build "Debug|Win32" "..\..\CHAI3D-VS2008.sln" > output.txt
goto done

:DebugForced
%cmp% /rebuild "Debug|Win32" "..\..\CHAI3D-VS2008.sln" > output.txt
goto done

:clean
%cmp% /clean "Debug|Win32"   "..\..\CHAI3D-VS2008.sln" > output.txt
%cmp% /clean "Release|Win32" "..\..\CHAI3D-VS2008.sln" > output.txt
goto done

:SKIP
echo     * warning - compiler missing, skipping
goto :EOF

:done
set PATH=%OLDPATH%

:EOF
