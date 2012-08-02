@echo off

REM  Software License Agreement (BSD License)
REM  Copyright (c) 2003-2012, CHAI3D.
REM  (www.chai3d.org)
REM
REM  All rights reserved.
REM
REM  Redistribution and use in source and binary forms, with or without
REM  modification, are permitted provided that the following conditions
REM  are met:
REM
REM  * Redistributions of source code must retain the above copyright
REM  notice, this list of conditions and the following disclaimer.
REM
REM  * Redistributions in binary form must reproduce the above
REM  copyright notice, this list of conditions and the following
REM  disclaimer in the documentation and/or other materials provided
REM  with the distribution.
REM
REM  * Neither the name of CHAI3D nor the names of its contributors may
REM  be used to endorse or promote products derived from this software
REM  without specific prior written permission.
REM
REM  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
REM  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
REM  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
REM  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
REM  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
REM  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
REM  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
REM  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
REM  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
REM  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
REM  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
REM  POSSIBILITY OF SUCH DAMAGE.
REM
REM  $Author: seb $
REM  $Date: 2012-05-07 20:36:47 +1200 (Mon, 07 May 2012) $
REM  $Rev: 832 $


set cmp=none
if exist "%ProgramFiles(x86)%\microsoft visual studio 10.0\common7\ide\devenv.exe"    set cmp=devenv
if exist "%ProgramFiles(x86)%\microsoft visual studio 10.0\common7\ide\VCExpress.exe" set cmp=VCExpress
if exist "%ProgramFiles%\microsoft visual studio 10.0\common7\ide\devenv.exe"         set cmp=devenv
if exist "%ProgramFiles%\microsoft visual studio 10.0\common7\ide\VCExpress.exe"      set cmp=VCExpress

if %cmp% == none goto SKIP

set OLDPATH=%PATH%
if exist "%ProgramFiles(x86)%\microsoft visual studio 10.0\common7\ide" set PATH="%ProgramFiles(x86)%\microsoft visual studio 10.0\common7\ide";%PATH%
if exist "%ProgramFiles%\microsoft visual studio 10.0\common7\ide" set PATH="%ProgramFiles%\microsoft visual studio 10.0\common7\ide";%PATH%

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
%cmp% /build "Release|x64" "..\..\CHAI3D-VS2010.sln" > output.txt
goto done

:ReleaseForced
%cmp% /rebuild "Release|x64" "..\..\CHAI3D-VS2010.sln"  > output.txt
goto done

:Debug
if %2a == a goto DebugBuild
if %2 == /f goto DebugForced
goto done

:DebugBuild
%cmp% /build "Debug|x64" "..\..\CHAI3D-VS2010.sln" > output.txt
goto done

:DebugForced
%cmp% /rebuild "Debug|x64" "..\..\CHAI3D-VS2010.sln" > output.txt
goto done

:clean
%cmp% /clean "Debug|x64"   "..\..\CHAI3D-VS2010.sln" > output.txt
%cmp% /clean "Release|x64" "..\..\CHAI3D-VS2010.sln" > output.txt
goto done

:SKIP
echo     * warning - compiler missing, skipping
goto :EOF

:done
set PATH=%OLDPATH%

:EOF
