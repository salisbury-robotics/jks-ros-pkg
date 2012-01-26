rem ===========================================================================
rem
rem    CHAI 3D    
rem    
rem    Author:	  Francois Conti
rem    		  conti@chai3d.org
rem    
rem    Purpose:	  This batch copies all resource file from the core directory
rem               to each release "bin"directory
rem
rem ===========================================================================

rem -----------------------------------------
rem INITIALISATION
rem -----------------------------------------

echo off

path
cd ../resources 


rem -----------------------------------------
rem COPY FILES
rem -----------------------------------------
xcopy /Y /S * ..\..\win32\bin\resources\
xcopy /Y /S * ..\..\win64\bin\resources\
xcopy /Y /S * ..\..\linux\bin\resources\
xcopy /Y /S * ..\..\mac\bin\resources\