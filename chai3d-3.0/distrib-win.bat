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
REM  \author    <http://www.chai3d.org>
REM  \author    Sebastien Grange
REM  \version   $Rev: 684 $


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
set TARGET_ARCHIVE=%TARGET%.zip
if exist %TARGET_DIR% rmdir /q /s %TARGET_DIR%
if exist %TARGET_TMP% rmdir /q /s %TARGET_TMP%
if exist %TARGET_ARCHIVE% del /q %TARGET_ARCHIVE%

echo.

echo test building VS2008 debug (win32)...
pushd projects\VS2008
call make debug /f
if ERRORLEVEL 1 goto error
popd

echo test building VS2008 release (win32)...
pushd projects\VS2008
call make /f
if ERRORLEVEL 1 goto error
popd

echo test building VS2008 debug (x64)...
pushd projects\VS2008
call make-x64 debug /f
if ERRORLEVEL 1 goto error
popd

echo test building VS2008 release (x64)...
pushd projects\VS2008
call make-x64 /f
if ERRORLEVEL 1 goto error
popd

echo test building VS2010 debug (win32)...
pushd projects\VS2010
call make debug /f
if ERRORLEVEL 1 goto error
popd

echo test building VS2010 release (win32)...
pushd projects\VS2010
call make /f
if ERRORLEVEL 1 goto error
popd

echo test building VS2010 debug (x64)...
pushd projects\VS2010
call make-x64 debug /f
if ERRORLEVEL 1 goto error
popd

echo test building VS2010 release (x64)...
pushd projects\VS2010
call make-x64 /f
if ERRORLEVEL 1 goto error
popd

echo.

echo exporting to temporary folder %TARGET_TMP%...
mkdir %TARGET_TMP%
if ERRORLEVEL 1 goto error
xcopy /T /E /Q /Y /EXCLUDE:utils\excludedir.txt . %TARGET_TMP% > log.txt 2>&1
if ERRORLEVEL 1 goto error
xcopy /E /Q /Y /EXCLUDE:utils\exclude.txt . %TARGET_TMP% > log.txt 2>&1
if ERRORLEVEL 1 goto error

echo relocating temporary folder locally...
move /Y %TARGET_TMP% %TARGET_DIR% > log.txt 2>&1
if ERRORLEVEL 1 goto error
pushd %TARGET_DIR%

echo replacing internal version info to %MAJOR%.%MINOR%.%RELEASE%.%REVISION%...
for /F "usebackq" %%f in (`utils\grep -rl "$MAJOR" .`)    do utils\sed -i "s/$MAJOR/%MAJOR%/g" %%f
if ERRORLEVEL 1 goto error
for /F "usebackq" %%f in (`utils\grep -rl "$MINOR" .`)    do utils\sed -i "s/$MINOR/%MINOR%/g" %%f
if ERRORLEVEL 1 goto error
for /F "usebackq" %%f in (`utils\grep -rl "$RELEASE" .`)  do utils\sed -i "s/$RELEASE/%RELEASE%/g" %%f
if ERRORLEVEL 1 goto error
for /F "usebackq" %%f in (`utils\grep -rl "$REVISION" .`) do utils\sed -i "s/$REVISION/%REVISION%/g" %%f
if ERRORLEVEL 1 goto error

echo compiling documentation...
pushd doc\resources\doxygen
doxygen Doxyfile > doc.txt 2>&1
if ERRORLEVEL 1 goto error
del /q doc.txt
popd

echo removing unwanted files from distribution folder...
rmdir /q /s utils > ..\log.txt 2>&1
if ERRORLEVEL 1 goto error
del /f /s sed* > ..\log.txt 2>&1
if ERRORLEVEL 1 goto error

echo exporting to archive %TARGET_ARCHIVE%...
popd
utils\zip -r %TARGET_ARCHIVE% %TARGET_DIR% > zip.txt 2>&1
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
