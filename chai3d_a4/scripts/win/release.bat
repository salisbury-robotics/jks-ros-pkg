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
REM  $Rev: 842 $


pushd ..\..

echo.
echo retrieving version info...
for /F "usebackq delims=: tokens=2" %%i in (`svnversion -c .`) do set REVISION=%%i
for /F "usebackq delims== tokens=2" %%i in (`findstr MAJOR .\src\version`) do set MAJOR=%%i
for /F "usebackq delims== tokens=2" %%i in (`findstr MINOR .\src\version`) do set MINOR=%%i
for /F "usebackq delims== tokens=2" %%i in (`findstr RELEASE .\src\version`) do set RELEASE=%%i
echo identified version %MAJOR%.%MINOR%.%RELEASE%.%REVISION%

echo cleaning up previous distribution attempts...
set TARGET=chai3d-%MAJOR%.%MINOR%.%RELEASE%
set TARGET_TMP=..\%TARGET%.%REVISION%-tmp
set TARGET_DIR=%TARGET%
set TARGET_ARCHIVE=releases\%TARGET%.zip
if exist %TARGET_DIR% rmdir /q /s %TARGET_DIR%
if exist %TARGET_TMP% rmdir /q /s %TARGET_TMP%
if exist %TARGET_ARCHIVE% del /q %TARGET_ARCHIVE%

echo.

REM skip compilation
if %1a == a  goto Build
if %1  == /s goto Skip

:Build

echo test building VS2008 debug (x64)...
pushd projects\VS2008
call make-x64 debug /f
if ERRORLEVEL 1 goto error
popd
echo test building VS2008 debug (x64) [GEL]...
pushd modules\GEL
call make-VS2008-x64 debug /f
if ERRORLEVEL 1 goto error
popd
echo test building VS2008 debug (x64) [ODE]...
pushd modules\ODE
call make-VS2008-x64 debug /f
if ERRORLEVEL 1 goto error
popd

echo test building VS2008 release (x64)...
pushd projects\VS2008
call make-x64 /f
if ERRORLEVEL 1 goto error
popd
echo test building VS2008 release (x64) [GEL]...
pushd modules\GEL
call make-VS2008-x64 /f
if ERRORLEVEL 1 goto error
popd
echo test building VS2008 release (x64) [ODE]...
pushd modules\ODE
call make-VS2008-x64 /f
if ERRORLEVEL 1 goto error
popd

echo test building VS2008 debug (win32)...
pushd projects\VS2008
call make debug /f
if ERRORLEVEL 1 goto error
popd
echo test building VS2008 debug (win32) [GEL]...
pushd modules\GEL
call make-VS2008 debug /f
if ERRORLEVEL 1 goto error
popd
echo test building VS2008 debug (win32) [ODE]...
pushd modules\ODE
call make-VS2008 debug /f
if ERRORLEVEL 1 goto error
popd

echo test building VS2008 release (win32)...
pushd projects\VS2008
call make /f
if ERRORLEVEL 1 goto error
popd
echo test building VS2008 release (win32) [GEL]...
pushd modules\GEL
call make-VS2008 /f
if ERRORLEVEL 1 goto error
popd
echo test building VS2008 release (win32) [ODE]...
pushd modules\ODE
call make-VS2008 /f
if ERRORLEVEL 1 goto error
popd

echo test building VS2010 debug (x64)...
pushd projects\VS2010
call make-x64 debug /f
if ERRORLEVEL 1 goto error
popd
echo test building VS2010 debug (x64) [GEL]...
pushd modules\GEL
call make-VS2010-x64 debug /f
if ERRORLEVEL 1 goto error
popd
echo test building VS2010 debug (x64) [ODE]...
pushd modules\ODE
call make-VS2010-x64 debug /f
if ERRORLEVEL 1 goto error
popd

echo test building VS2010 release (x64)...
pushd projects\VS2010
call make-x64 /f
if ERRORLEVEL 1 goto error
popd
echo test building VS2010 release (x64) [GEL]...
pushd modules\GEL
call make-VS2010-x64 /f
if ERRORLEVEL 1 goto error
popd
echo test building VS2010 release (x64) [ODE]...
pushd modules\ODE
call make-VS2010-x64 /f
if ERRORLEVEL 1 goto error
popd

echo test building VS2010 debug (win32)...
pushd projects\VS2010
call make debug /f
if ERRORLEVEL 1 goto error
popd
echo test building VS2010 debug (win32) [GEL]...
pushd modules\GEL
call make-VS2010 /f
if ERRORLEVEL 1 goto error
popd
echo test building VS2010 debug (win32) [ODE]...
pushd modules\ODE
call make-VS2010 /f
if ERRORLEVEL 1 goto error
popd

echo test building VS2010 release (win32)...
pushd projects\VS2010
call make /f
if ERRORLEVEL 1 goto error
popd
echo test building VS2010 release (win32) [GEL]...
pushd modules\GEL
call make-VS2010 /f
if ERRORLEVEL 1 goto error
popd
echo test building VS2010 release (win32) [ODE]...
pushd modules\ODE
call make-VS2010 /f
if ERRORLEVEL 1 goto error
popd

echo.

echo compiling documentation...
pushd doc\doxygen
doxygen Doxyfile > doc.txt 2>&1
if ERRORLEVEL 1 goto error
del /q doc.txt
popd

goto Skipped

:Skip
echo skipping build on user request
echo.

:Skipped

echo exporting to temporary folder %TARGET_TMP%...
mkdir %TARGET_TMP%
if ERRORLEVEL 1 goto error
for /f %%a in ('type .\scripts\win\folders.txt') do (
  echo creating %TARGET_TMP%\%%a > log.txt 2>&1
  mkdir %TARGET_TMP%\%%a >> log.txt 2>&1
  if ERRORLEVEL 1 goto error
)
if ERRORLEVEL 1 goto error
for /f %%a in ('type .\scripts\win\files.txt') do (
  echo copying %%a to %TARGET_TMP%\%%a > log.txt 2>&1
  copy %%a %TARGET_TMP%\%%a >> log.txt 2>&1
  if ERRORLEVEL 1 goto error
)

echo relocating temporary folder locally...
move /Y %TARGET_TMP% %TARGET_DIR% > log.txt 2>&1
if ERRORLEVEL 1 goto error
pushd %TARGET_DIR%

echo replacing internal version info to %MAJOR%.%MINOR%.%RELEASE%.%REVISION%...
for /F "usebackq" %%f in (`scripts\win\grep -rl "$MAJOR" .`)    do scripts\win\sed -i "s/$MAJOR/%MAJOR%/g" %%f
if ERRORLEVEL 1 goto error
for /F "usebackq" %%f in (`scripts\win\grep -rl "$MINOR" .`)    do scripts\win\sed -i "s/$MINOR/%MINOR%/g" %%f
if ERRORLEVEL 1 goto error
for /F "usebackq" %%f in (`scripts\win\grep -rl "$RELEASE" .`)  do scripts\win\sed -i "s/$RELEASE/%RELEASE%/g" %%f
if ERRORLEVEL 1 goto error
for /F "usebackq" %%f in (`scripts\win\grep -rl "$REVISION" .`) do scripts\win\sed -i "s/$REVISION/%REVISION%/g" %%f
if ERRORLEVEL 1 goto error

echo removing unwanted files from distribution folder...
rmdir /q /s scripts > ..\log.txt 2>&1
if ERRORLEVEL 1 goto error
del /f /s sed* > ..\log.txt 2>&1
if ERRORLEVEL 1 goto error

echo exporting to archive %TARGET_ARCHIVE%...
popd
mkdir releases > log.txt 2>&1
scripts\win\zip -r %TARGET_ARCHIVE% %TARGET_DIR% > zip.txt 2>&1
if ERRORLEVEL 1 goto error
del /q zip.txt

echo cleaning up temporary folder...
rmdir /q /s %TARGET_DIR% > log.txt 2>&1
if ERRORLEVEL 1 goto error

:done
echo.
echo %TARGET% release created successfully
del /q log.txt
goto EOF

:error
echo   *** error - check log files for more information
pause
EXIT /B 1

:EOF
popd
